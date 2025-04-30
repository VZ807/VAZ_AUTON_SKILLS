#include "lemlib/api.hpp"
#include "roboconfig.hpp"
class PID {
public:
  /**
   * A pid controlled turn, the default values are provided in the function def,
   * only pass arguments if nessesary
   * @param target
   *       target heading
   * @param hkP
   *       Proportional value
   * @param hkI
   *       Integral Value
   * @param hkP
   *       Derivitive Value
   *
   */
  void turn(double target, double timeout = 500000, double cap = 400,
            double hkP = 1.2, double hkI = 0.0, double hkD = .5);

  /**
   * A pid controlled drive, the default values are provided in the function
   * def, only pass arguments if nessesary
   * @param target
   *       target heading
   * @param timeout
   *       settletime input
   * @param cap
   *       max and min velocity
   * @param hkP
   *       Proportional value
   * @param hkI
   *       Integral Value
   * @param hkP
   *       Derivitive Value
   *
   */
  void drive(double target, double timeout = 500000, double cap = 400,
             double hkP = 150, double hkI = .7, double hkD = 95);
};
