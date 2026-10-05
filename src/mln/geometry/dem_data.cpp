#include <mln/geometry/dem_data.hpp>
#include <mln/math/clamp.hpp>

#include <algorithm>
#include <limits>

namespace mln {

namespace {
// Terrarium encodes "no data" as RGB(0, 0, 0), which unpacks to this elevation.
constexpr int32_t terrariumNoData = -32768;
} // namespace

DEMData::DEMData(const PremultipliedImage& _image, Tileset::RasterEncoding _encoding)
    : dim(_image.size.height),
      // extra two pixels per row for border backfilling on either edge. Two, not one, so a
      // bilinear fetch centred on an edge pixel still has a full neighbourhood - and so the
      // hillshade prepare pass can derive a 1px ring outside the tile, which needs DEM
      // samples two pixels out. Matches maplibre-gl-js #8302.
      stride(dim + 4),
      encoding(_encoding) {
    image = std::make_shared<PremultipliedImage>(Size(static_cast<uint32_t>(stride), static_cast<uint32_t>(stride)));
    if (_image.size.height != _image.size.width) {
        throw std::runtime_error("raster-dem tiles must be square.");
    }

    auto* dest = reinterpret_cast<uint32_t*>(image->data.get()) + stride * 2 + 2;
    auto* source = reinterpret_cast<uint32_t*>(_image.data.get());
    for (int32_t y = 0; y < dim; y++) {
        memcpy(dest, source, dim * 4);
        dest += stride;
        source += dim;
    }

    // in order to avoid flashing seams between tiles, here we are initially
    // populating a 2px border of pixels around the image with the data of the
    // nearest pixel from the image. this data is eventually replaced when the
    // tile's neighboring tiles are loaded and the accurate data can be
    // backfilled using DEMData#backfillBorder

    auto* data = reinterpret_cast<uint32_t*>(image->data.get());
    if (dim > 0) {
        for (int32_t y = -2; y < dim + 2; y++) {
            auto* src = data + idx(0, std::clamp(y, 0, dim - 1));
            auto* dst = data + idx(0, y);
            if (y < 0 || y >= dim) {
                memcpy(dst, src, dim * 4);
            }
            dst[-2] = dst[-1] = src[0];
            dst[dim] = dst[dim + 1] = src[dim - 1];
        }
    }

    // The elevation range of the tile, and of its sub-rectangles, gives a tile a height when
    // testing it against the view frustum (see util::tileCover): terrain rising towards the
    // camera is visible from further away than its flat footprint suggests. Computed once
    // here, on the worker thread that decodes the tile, rather than per frame. The border is
    // excluded: it is a copy of the edge pixels until neighbouring tiles backfill it, so it
    // holds no elevation this tile does not already have.
    buildMinMaxPyramid();
}

void DEMData::buildMinMaxPyramid() {
    if (dim <= 0) {
        return;
    }
    constexpr int32_t finestCellSize = 4;
    MinMaxLevel level{finestCellSize, (dim + finestCellSize - 1) / finestCellSize, {}, {}};
    level.min.assign(static_cast<size_t>(level.cells) * level.cells, std::numeric_limits<int32_t>::max());
    level.max.assign(level.min.size(), std::numeric_limits<int32_t>::lowest());
    for (int32_t y = 0; y < dim; y++) {
        for (int32_t x = 0; x < dim; x++) {
            const int32_t value = get(x, y);
            const size_t cell = static_cast<size_t>(y / finestCellSize) * level.cells + x / finestCellSize;
            level.min[cell] = std::min(level.min[cell], value);
            level.max[cell] = std::max(level.max[cell], value);
        }
    }
    minMaxPyramid.push_back(std::move(level));

    while (minMaxPyramid.back().cells > 1) {
        const MinMaxLevel& fine = minMaxPyramid.back();
        MinMaxLevel coarse{fine.cellSize * 2, (fine.cells + 1) / 2, {}, {}};
        coarse.min.assign(static_cast<size_t>(coarse.cells) * coarse.cells, std::numeric_limits<int32_t>::max());
        coarse.max.assign(coarse.min.size(), std::numeric_limits<int32_t>::lowest());
        for (int32_t y = 0; y < fine.cells; y++) {
            for (int32_t x = 0; x < fine.cells; x++) {
                const size_t from = static_cast<size_t>(y) * fine.cells + x;
                const size_t to = static_cast<size_t>(y / 2) * coarse.cells + x / 2;
                coarse.min[to] = std::min(coarse.min[to], fine.min[from]);
                coarse.max[to] = std::max(coarse.max[to], fine.max[from]);
            }
        }
        minMaxPyramid.push_back(std::move(coarse));
    }

    minElevation = minMaxPyramid.back().min[0];
    maxElevation = minMaxPyramid.back().max[0];
}

Range<int32_t> DEMData::getElevationRange(int32_t x0, int32_t y0, int32_t x1, int32_t y1) const {
    if (minMaxPyramid.empty()) {
        return {minElevation, maxElevation};
    }
    x0 = std::clamp(x0, 0, dim - 1);
    y0 = std::clamp(y0, 0, dim - 1);
    x1 = std::clamp(x1, x0, dim - 1);
    y1 = std::clamp(y1, y0, dim - 1);

    // The finest level whose cells are at least half the rectangle wide, so the
    // rectangle touches at most three cells per axis.
    const int32_t extent = std::max(x1 - x0, y1 - y0) + 1;
    const MinMaxLevel* level = &minMaxPyramid.back();
    for (const auto& candidate : minMaxPyramid) {
        if (candidate.cellSize * 2 >= extent) {
            level = &candidate;
            break;
        }
    }

    int32_t lo = std::numeric_limits<int32_t>::max();
    int32_t hi = std::numeric_limits<int32_t>::lowest();
    for (int32_t cy = y0 / level->cellSize; cy <= y1 / level->cellSize; cy++) {
        for (int32_t cx = x0 / level->cellSize; cx <= x1 / level->cellSize; cx++) {
            const size_t cell = static_cast<size_t>(cy) * level->cells + cx;
            lo = std::min(lo, level->min[cell]);
            hi = std::max(hi, level->max[cell]);
        }
    }
    return {lo, hi};
}

// This function takes the DEMData from a neighboring tile and backfills the
// edge/corner data in order to create a two pixel "buffer" of image data around
// the tile. This is necessary because the hillshade formula calculates the
// dx/dz, dy/dz derivatives at each pixel of the tile by querying the 8
// surrounding pixels, and if we don't have the pixel buffer we get seams at
// tile boundaries.
void DEMData::backfillBorder(const DEMData& borderTileData, int8_t dx, int8_t dy) {
    auto& o = borderTileData;

    // Tiles from the same source should always be of the same dimensions.
    assert(dim == o.dim);

    // We determine the pixel range to backfill based which corner/edge
    // `borderTileData` represents. For example, dx = -1, dy = -1 represents the
    // upper left corner of the base tile, so we only need to backfill one pixel
    // at coordinates (-1, -1) of the tile image.
    int32_t xMin = dx * dim;
    int32_t xMax = dx * dim + dim;
    int32_t yMin = dy * dim;
    int32_t yMax = dy * dim + dim;

    if (dx == -1)
        xMin = xMax - 2;
    else if (dx == 1)
        xMax = xMin + 2;

    if (dy == -1)
        yMin = yMax - 2;
    else if (dy == 1)
        yMax = yMin + 2;

    int32_t ox = -dx * dim;
    int32_t oy = -dy * dim;

    auto* dest = reinterpret_cast<uint32_t*>(image->data.get());
    auto* source = reinterpret_cast<uint32_t*>(o.image->data.get());

    for (int32_t y = yMin; y < yMax; y++) {
        for (int32_t x = xMin; x < xMax; x++) {
            dest[idx(x, y)] = source[idx(x + ox, y + oy)];
        }
    }
}

int32_t DEMData::get(const int32_t x, const int32_t y) const {
    const auto& unpack = getUnpackVector();
    const uint8_t* value = image->data.get() + idx(x, y) * 4;
    const auto elevation = static_cast<int32_t>(value[0] * unpack[0] + value[1] * unpack[1] + value[2] * unpack[2] -
                                                unpack[3]);
    // Reading no-data as -32768 m would drag the tile's elevation range (and the frustum
    // test built on it) down with it.
    if (encoding == Tileset::RasterEncoding::Terrarium && elevation == terrariumNoData) {
        return 0;
    }
    return elevation;
}

const std::array<float, 4>& DEMData::getUnpackVector() const {
    // https://www.mapbox.com/help/access-elevation-data/#mapbox-terrain-rgb
    static const std::array<float, 4> unpackMapbox = {{6553.6f, 25.6f, 0.1f, 10000.0f}};
    // https://aws.amazon.com/public-datasets/terrain/
    static const std::array<float, 4> unpackTerrarium = {{256.0f, 1.0f, 1.0f / 256.0f, 32768.0f}};

    return encoding == Tileset::RasterEncoding::Terrarium ? unpackTerrarium : unpackMapbox;
}

} // namespace mln
