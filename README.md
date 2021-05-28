# ADE77880 Three-phase Power Sensor for ESPHome
The ade7880 sensor platform allows you to use ADE7880 three phase energy metering ICs [(datasheet)](https://www.analog.com/media/en/technical-documentation/data-sheets/ADE7880.pdf) with ESPHome. These are commonly found in Shelly 3EM devices.

The ADE7880 is suitable for measuring active, reactive, and apparent energy in various 3-phase configurations, such as wye  or delta services with, both, three and four wires. The ADE7880 provides system calibration features for each phase, that is, rms offset correction, phase calibration, and gain calibration. The CF1, CF2, and CF3 logic outputs provide a wide choice of  power information: total active powers, apparent powers, or the sum of the current rms values, and fundamental active and reactive powers.

## Types of three-phases
The ADE7880 is capable of measuring different types of 3-phase configurations: 3-wire delta, 4-wire wye. More on this in the [application note AN-639 (section METER CONFIGURATIONS)](https://www.analog.com/media/en/technical-documentation/application-notes/AN-639.pdf?doc=ADE7880.pdf).
