#include <mln/test/util.hpp>
#include <mln/test/stub_file_source.hpp>
#include <mln/test/map_adapter.hpp>

#include <mln/gfx/headless_frontend.hpp>
#include <mln/map/camera.hpp>
#include <mln/map/map_options.hpp>
#include <mln/style/style.hpp>
#include <mln/util/image.hpp>
#include <mln/util/run_loop.hpp>
#include <mln/util/chrono.hpp>

#include <thread>

using namespace mln;

namespace {

constexpr uint32_t demTileSize = 64;
// Terrain-RGB encodes elevation as -10000 + (R * 65536 + G * 256 + B) * 0.1, so a flat
// 1000 m plateau is RGB(1, 173, 176): 65536 + 44288 + 176 = 110000.
constexpr double plateauMeters = 1000.0;

std::string makeFlatDEMTile(uint8_t r = 1, uint8_t g = 173, uint8_t b = 176) {
    PremultipliedImage image({demTileSize, demTileSize});
    for (uint32_t i = 0; i < demTileSize * demTileSize; ++i) {
        uint8_t* px = image.data.get() + i * 4;
        px[0] = r;
        px[1] = g;
        px[2] = b;
        px[3] = 255;
    }
    return encodePNG(image);
}

const char* terrainStyle = R"STYLE({
  "version": 8,
  "sources": {
    "dem": {
      "type": "raster-dem",
      "tiles": ["http://example.com/{z}-{x}-{y}.png"],
      "encoding": "mapbox",
      "maxzoom": 2,
      "tileSize": 64
    }
  },
  "terrain": {"source": "dem", "exaggeration": 1.0},
  "layers": []
})STYLE";

struct TerrainCameraTest {
    util::RunLoop loop;
    std::shared_ptr<StubFileSource> fileSource = std::make_shared<StubFileSource>(ResourceOptions::Default(),
                                                                                  ClientOptions());
    std::string tile = makeFlatDEMTile();
    HeadlessFrontend frontend{{256, 256}, 1};
    MapAdapter map;

    explicit TerrainCameraTest(MapMode mode = MapMode::Static)
        : map(frontend,
              MapObserver::nullObserver(),
              fileSource,
              MapOptions().withMapMode(mode).withSize(frontend.getSize())) {
        fileSource->tileResponse = [this](const Resource&) {
            Response res;
            res.data = std::make_shared<std::string>(tile);
            return res;
        };
        map.getStyle().loadJSON(terrainStyle);
        map.jumpTo(CameraOptions().withCenter(LatLng{47.2692, 11.4041}).withZoom(2.0).withPitch(60.0));
    }

    void renderFrame() {
        loop.runOnce();
        if (map.getMapOptions().mapMode() == MapMode::Continuous) {
            frontend.renderOnce(map);
        } else {
            frontend.render(map);
        }
    }

    double renderAndGetAltitude(int frames) {
        double altitude = 0.0;
        for (int i = 0; i < frames; ++i) {
            renderFrame();
            altitude = map.getCameraOptions({}).centerAltitude.value_or(0.0);
        }
        return altitude;
    }
};

} // namespace

// The centre rides the terrain instead of sea level, so pitching over high ground does not
// leave the camera inside the hillside.
TEST(TerrainCamera, CentreRidesTheTerrainSurface) {
    TerrainCameraTest test;
    EXPECT_TRUE(test.map.getCenterClampedToGround());

    const double settled = test.renderAndGetAltitude(8);
    EXPECT_NEAR(settled, plateauMeters, 5.0);
}

// The historical worry about a terrain-anchored centre was that raising it re-samples a new
// elevation and runs away. Raising the centre moves the orbit plane, not the centre's lng/lat,
// so it must settle - and the centre must not drift while it does.
TEST(TerrainCamera, CentreElevationSettlesWithoutDrift) {
    TerrainCameraTest test;

    test.renderAndGetAltitude(8);
    const auto afterSettling = test.map.getCameraOptions({});
    const double settledAltitude = *afterSettling.centerAltitude;
    // Guard against settling "stably" at sea level, which would make the rest vacuous.
    ASSERT_NEAR(settledAltitude, plateauMeters, 5.0);

    // Ten more frames with nothing else changing.
    const double later = test.renderAndGetAltitude(10);
    const auto atEnd = test.map.getCameraOptions({});

    EXPECT_NEAR(later, settledAltitude, 0.5);
    EXPECT_NEAR(atEnd.center->latitude(), afterSettling.center->latitude(), 1e-6);
    EXPECT_NEAR(atEnd.center->longitude(), afterSettling.center->longitude(), 1e-6);
    EXPECT_NEAR(*atEnd.zoom, *afterSettling.zoom, 1e-6);
}

// Turning it off leaves the camera where the caller put it.
TEST(TerrainCamera, ClampingCanBeTurnedOff) {
    TerrainCameraTest test;
    test.map.setCenterClampedToGround(false);
    EXPECT_FALSE(test.map.getCenterClampedToGround());

    EXPECT_NEAR(test.renderAndGetAltitude(8), 0.0, 0.001);
}

// A clamp report during an animation must not end it. Each report did a jumpTo, and starting
// that instant transition finished the running one where it stood: a fly-to over terrain
// stopped in its first frames, short of its target.
TEST(TerrainCamera, ClampLeavesARunningAnimationToFinish) {
#if MLN_RENDER_BACKEND_METAL
    // Continuous headless rendering stalls on Metal on this branch, waiting for a command
    // buffer (Map.ContinuousRendering times out the same way); the test needs continuous
    // frames, so it runs on the other backends.
    GTEST_SKIP() << "continuous headless rendering stalls on Metal";
#endif
    // Continuous: a still render runs every transition to its end before drawing.
    TerrainCameraTest test{MapMode::Continuous};
    test.map.setCenterClampedToGround(true);

    // Tiles west of the antimeridian's opposite, x = 0 and 1 at z2, stand at 2000 m:
    // -10000 + (1 * 65536 + 212 * 256 + 192) * 0.1. Flying onto them makes the
    // clamp report mid-flight.
    const std::string higherTile = makeFlatDEMTile(1, 212, 192);
    test.fileSource->tileResponse = [&](const Resource& resource) {
        Response res;
        res.data = std::make_shared<std::string>(resource.tileData->x < 2 ? higherTile : test.tile);
        return res;
    };
    ASSERT_NEAR(test.renderAndGetAltitude(8), plateauMeters, 5.0);

    const LatLng target{47.2692, -20.0};
    AnimationOptions animation;
    animation.duration = Milliseconds(2000);
    test.map.easeTo(CameraOptions().withCenter(target), animation);
    for (int i = 0; i < 16; ++i) {
        test.renderFrame();
        std::this_thread::sleep_for(Milliseconds(100));
    }

    const CameraOptions camera = test.map.getCameraOptions({});
    EXPECT_NEAR(camera.center->longitude(), target.longitude(), 1e-6);
    EXPECT_NEAR(camera.center->latitude(), target.latitude(), 1e-6);
    // ...and the clamp resumes once the animation is over.
    EXPECT_NEAR(*camera.centerAltitude, 2000.0, 5.0);
}
