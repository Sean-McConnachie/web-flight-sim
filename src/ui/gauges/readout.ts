/**
 * What the cockpit instruments read.
 *
 * ONE INTERFACE, AND ONLY ONE
 *
 * Every value that reaches a dial arrives through `TelemetrySample` of
 * src/ui/debug-overlay.ts or through `CockpitReadout` below. No file under
 * src/ui/gauges imports src/physics or src/aircraft, and no file under
 * src/ui/gauges reads a global. The composition root fills both records one
 * time per frame. The types are STRUCTURAL, so they name the fields the dials
 * read and they carry no physics type.
 *
 * `TelemetrySample` already holds the airspeed, the height, the attitude, the
 * climb rate and the air. The dials reuse it. `CockpitReadout` adds only what
 * that sample does not carry: the two engines, the fuel, and the two specific
 * force components that drive the slip ball and the gyro erection error.
 *
 * UNITS
 *
 * Every value below is SI, as CONVENTIONS section 2 demands. Rotor speed is
 * radians per second here and becomes rpm only inside the tachometer module,
 * through `radPerSecToRpm` of src/math/units.ts. The same rule holds for the
 * gas temperature, which is kelvin here and degrees Celsius on the face.
 */

/** What one dial set reads from one Jumo 004. */
export interface EngineGaugeReadout {
  /** Rotor speed, rad/s. The Drehzahlmesser converts it to rpm. */
  readonly rotorSpeed: number;
  /** Gas temperature at the turbine inlet, K. */
  readonly gasTemperature: number;
  /** Fuel flow into the six chambers, kg/s. */
  readonly fuelFlow: number;
}

/**
 * What the panel reads from the aircraft, less the part that the telemetry
 * sample already carries.
 */
export interface CockpitReadout {
  /** One record per engine, in the order left engine first. */
  readonly engines: readonly EngineGaugeReadout[];
  /** Fuel on board, kg. */
  readonly fuelMass: number;
  /**
   * Body y specific force, m/s2, with gravity taken out. Positive points out
   * of the starboard wing. The slip ball hangs on this value and on nothing
   * else. A rudder deflection reaches the ball only through the side force it
   * makes, which is already inside this number.
   */
  readonly lateralAcceleration: number;
  /**
   * Body x specific force, m/s2, with gravity taken out. Positive points out
   * of the nose. The pendulous vanes of the gyro horizon feel it, so it drives
   * the false climb error of the Wendehorizont.
   */
  readonly longitudinalAcceleration: number;
}
