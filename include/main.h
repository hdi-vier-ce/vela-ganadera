#include <Arduino.h>
#include "axp20x.h"
#include <vector>
String getBaChStatus();
void setBaChStatus(String value);
bool getSsd1306_found();
bool getAxp192_found();
AXP20X_Class getAxp();
bool getPmu_irq();
void setPmu_irq(bool value);
int voltageToPercentage ();
