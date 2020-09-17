# AHRS

Extended Kalman Filter for Attitude and Heading Reference System

# Usage

```shell
Core
├── Inc
│   ├── ahrs.h
│   └── ...
├── Src
│   ├── ahrs.cpp
│   └── ...
└── ...
```

```cpp
...

/* USER CODE BEGIN Includes */
#include "ahrs.h"

#include <stdio.h>
/* USER CODE END Includes */

...

int main(void) {

    ...

    /* USER CODE BEGIN 2 */
    lot::AHRS ahrs;
    /* USER CODE END 2 */

    ...

    /* USER CODE BEGIN WHILE */
    while(1) {

        ...

        ahrs.predict(half_delta_angle);

        float unit_acc_cross_mag[3];
        ahrs.get_cross_product(unit_acc, mag_xyz, unit_acc_cross_mag);
        ahrs.get_unit_vector(unit_acc_cross_mag, 3);
        ahrs.update(unit_acc, unit_acc_cross_mag);
        /* USER CODE END WHILE */

        float rpy[3];
        ahrs.get_RPY(rpy);
        printf("%.2f,%.2f,%.2f\n", rpy[0], rpy[1], rpy[2]);

        float predicted_gravity[3] = {0.0, 0.0, 1.0};
        ahrs.get_frame_rotation(predicted_gravity);
        ahrs.get_unit_vector(predicted_gravity, 3);
        ...

    }

    ...

}

...
```
