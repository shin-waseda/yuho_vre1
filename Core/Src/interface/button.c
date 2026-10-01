#include "main.h"
#include "interface/button.h"

bool Button_IsPressed(void) {
    return HAL_GPIO_ReadPin(Push_IN_1_GPIO_Port, Push_IN_1_Pin) == GPIO_PIN_RESET;
}
