/**
 * Tunable values.
 *
 * One file holds every number that a person tunes by hand. A module reads the
 * group that belongs to it. A module never copies a value into its own file.
 *
 * All values use SI units. Read docs/CONVENTIONS.md section 2.
 */

export const config = {
  /**
   * src/core/loop.ts reads this group. src/physics reads the gravity value
   * through the caller that builds the wrench.
   */
  physics: {
    /** Physics steps per second. */
    rate: 240,

    /**
     * Largest real time the loop simulates in one frame, in seconds.
     * The cap stops a death spiral after a slow frame or a background tab.
     */
    accumulatorCap: 0.25,

    /**
     * Standard gravity, in m/s^2. It acts on the world z axis, which points
     * down. Source: CGPM 1901 standard value, confidence: firm. The value
     * matches G0 in src/math/units.ts.
     */
    gravity: 9.80665,
  },

  /** src/world builds the ground, the runway, and the scatter from this group. */
  world: {
    /** Side of the square ground plane, in meters. */
    groundSize: 40000,

    /**
     * Runway length, in meters. The Me-262 needs about 1100 m at the maximum
     * takeoff mass, so this length leaves room to stop.
     */
    runwayLength: 2400,

    /** Runway width, in meters. */
    runwayWidth: 45,

    /** Seed for src/core/prng.ts. The same seed rebuilds the same world. */
    scatterSeed: 262,

    /** Number of trees the scatter places outside the airfield. */
    treeCount: 4000,

    /** Number of buildings the scatter places outside the airfield. */
    buildingCount: 120,
  },

  /** src/render reads this group. */
  render: {
    shadowsEnabled: true,

    /** Side of the shadow map, in texels. */
    shadowMapSize: 2048,

    /** Distance from the camera that still receives a shadow, in meters. */
    shadowDistance: 600,

    /**
     * Depth bias of the shadow map. A negative value hides shadow acne.
     *
     * The unit is normalized shadow camera depth, not meters. The cascade
     * shadow node also multiplies the bias by the cascade index. Over the
     * 1 m to 1500 m shadow camera range, a value of -0.0005 moves the shadow
     * about 0.37 m on the first cascade and about 1.1 m on the third. The
     * aircraft would then appear to float above its own shadow.
     */
    shadowBias: -0.00005,

    /**
     * Tone mapping exposure of the renderer.
     *
     * The scene holds no physical light unit. src/render/sky.ts takes its
     * radiance scale from `SkyMesh`, which multiplies the Preetham result by
     * 0.04. The sun light comes from the same scale, because it is the radiance
     * of the solar disc times its solid angle. So the sky, the sun, and the fog
     * all agree with each other, and one number ties the whole set to the
     * screen. That number is this exposure. Do not fix a bright picture by a
     * change of the sun intensity, because that would break the agreement.
     *
     * The value comes from a grey card. At a sun elevation of 45 degrees the
     * irradiance on flat ground is 12.96 from the sun and 3.46 from the sky
     * hemisphere, so 16.4 in total, in the green channel. A Lambertian surface
     * of 18 percent reflectance then sends out 0.18 * 16.4 / pi = 0.94.
     *
     * The ACES curve of Three.js must return that surface to the screen value
     * that 18 percent linear holds in sRGB, which is 0.46, or 118 of 255. An
     * exposure of 0.17 does this. A grey card reads 119, sunlit grass reads
     * near 87, fresh runway paint reads near 212, and nothing clips.
     */
    exposure: 0.17,

    /** Distance where the fog starts, in meters. */
    fogNear: 3000,

    /** Distance where the fog hides the ground, in meters. */
    fogFar: 30000,
  },

  /** src/input shapes the raw axis values with this group. */
  input: {
    /** Axis values below this size read as zero. */
    deadZone: 0.06,

    /**
     * Curve strength of the axis map, from 0 to 1. A value of 0 keeps the axis
     * linear. A larger value makes the center of the axis less sensitive.
     */
    expo: 0.4,

    /**
     * Time for the throttle lever to travel from closed to full, in seconds.
     * A digital button gives no lever position, so src/input/bindings.ts turns
     * a button press into a rate.
     *
     * This value shapes the INPUT DEVICE. It is not the engine spool model.
     * The Jumo 004 rotor inertia and the surge that a fast lever causes live in
     * src/aircraft/me262/engine.ts. The engine must punish a pilot who slams
     * the lever. It must not punish a pilot who holds a digital button.
     */
    throttleSweepTime: 2,
  },
} as const;

export type Config = typeof config;
