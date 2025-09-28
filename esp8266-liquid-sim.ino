#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <FastLED.h>
#include "fl/leds.h"
#include "fl/xymap.h"

constexpr uint16_t WIDTH = 8;
constexpr uint16_t HEIGHT = 8;
constexpr uint16_t NUM_LEDS = WIDTH * HEIGHT;
CRGB leds[NUM_LEDS];

// MPU6050
Adafruit_MPU6050 mpu;

// Timing
const float dt = 1.0f / 30.0f;  // 30 Hz update

// Particle definitions
struct vec {
  float x, y;
};

struct Particle {
  vec X;  // Position
  vec V;  // Velocity
};

const int Num_of_particles = 20;
std::vector<Particle> particles;

// Gravity vector
vec g;

// Setup
void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  FastLED.addLeds<WS2812B, 2, GRB>(leds, NUM_LEDS);
  FastLED.clear();

  // Create particles in random spots
  for (int i = 0; i < Num_of_particles; i++) {
    Particle p;
    p.X = { random(0, WIDTH), random(0, HEIGHT) };
    p.V = { 0.0f, 0.0f };
    particles.push_back(p);
  }
}

// Utility: normalize vector
vec normalize(vec v) {
  float mag = sqrt(v.x * v.x + v.y * v.y);
  if (mag > 0.0001f) {
    return { v.x / mag, v.y / mag };
  }
  return { 0, 0 };
}

// Utility: subtract vectors
vec sub(vec a, vec b) {
  return { a.x - b.x, a.y - b.y };
}

// Loop
void loop() {
  // Get accelerometer data
  sensors_event_t a, gyro, temp;
  mpu.getEvent(&a, &gyro, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitchRad = atan2(ax, sqrt(ay * ay + az * az));
  float rollRad = atan2(ay, sqrt(ax * ax + az * az));

  // Update gravity vector (projected into 2D plane of LED grid)
  g.x = 9.81f * sin(rollRad);
  g.y = -9.81f * sin(pitchRad);

  // Build LED mapping
  auto map = fl::XYMap::constructSerpentine(WIDTH, HEIGHT);
  fl::Leds s = fl::Leds(leds, map);

  // Clear screen
  FastLED.clear();

  // Update particles
  for (int i = 0; i < Num_of_particles; i++) {
    // Update velocity
    particles[i].V.x += dt * g.x;
    particles[i].V.y += dt * g.y;


    // Update position
    particles[i].X.x += dt * particles[i].V.x;
    particles[i].X.y += dt * particles[i].V.y;

    // Bounce at edges
    if (particles[i].X.x < 0) {
      particles[i].X.x = 0;
      particles[i].V.x *= -1.0f;
    }
    if (particles[i].X.x > WIDTH - 1) {
      particles[i].X.x = WIDTH - 1;
      particles[i].V.x *= -1.0f;
    }
    if (particles[i].X.y < 0) {
      particles[i].X.y = 0;
      particles[i].V.y *= -1.0f;
    }
    if (particles[i].X.y > HEIGHT - 1) {
      particles[i].X.y = HEIGHT - 1;
      particles[i].V.y *= -1.0f;
    }
  }

  // Resolve particle–particle collisions (incompressibility)
  float minDist = 0.9f;  // particles occupy ~1 LED cell
  for (int i = 0; i < Num_of_particles; i++) {
    for (int j = i + 1; j < Num_of_particles; j++) {
      vec diff = sub(particles[j].X, particles[i].X);
      float dist = sqrt(diff.x * diff.x + diff.y * diff.y);

      if (dist < minDist && dist > 0.0001f) {
        vec n = normalize(diff);
        float overlap = 1.0f * (minDist - dist);

        // Push particles apart
        particles[i].X.x -= overlap * n.x;
        particles[i].X.y -= overlap * n.y;
        particles[j].X.x += overlap * n.x;
        particles[j].X.y += overlap * n.y;

        // Exchange some velocity along collision normal
        float vi = particles[i].V.x * n.x + particles[i].V.y * n.y;
        float vj = particles[j].V.x * n.x + particles[j].V.y * n.y;

        float avg = (vi + vj) / 2.0f;

        particles[i].V.x += (avg - vi) * n.x;
        particles[i].V.y += (avg - vi) * n.y;
        particles[j].V.x += (avg - vj) * n.x;
        particles[j].V.y += (avg - vj) * n.y;
      }
    }
  }

  // Draw particles
  for (int i = 0; i < Num_of_particles; i++) {
    int px = int(particles[i].X.x);
    int py = int(particles[i].X.y);

    px = constrain(px, 0, WIDTH - 1);
    py = constrain(py, 0, HEIGHT - 1);

    auto xymap = fl::XYMap::constructRectangularGrid(WIDTH, HEIGHT);
    fl::Leds s(leds, xymap);
    s(px, py) = CHSV(100, 255, 100);
  }

  FastLED.show();
}
