## The Vision Control Loop

The Python script (`camera_worker.py`) constantly runs the following feedback loop for every single frame of the video:

### 1. Setpoint (Target)
The goal is to keep the **detected black line** exactly in the center of the camera's view. The camera image is `W` pixels wide, our setpoint is `W / 2`.

### 2. Measurement (Actual Value)
OpenCV filters the image, applies binary thresholding, and finds the **black track**. It calculates the center of mass of this object. The X-coordinate of this centroid is our actual position (`cx`).

### 3. Error Calculation
The system calculates the distance between where the line *is* and where we *want* it to be:

```text
Error = (W / 2) - cx
```

### 4. Regulator

The error is multiplied by a tuning constant (Proportional Gain, `kp_angular`) to generate the steering command:
   `Angular Speed = kp_angular * Error`
   *A larger error results in a sharper turn. As the robot centers itself over the line, the error approaches zero, and the turning command smoothly decreases.*

### 5. executionr

The calculated `Angular Speed` (along with a constant `Linear Speed` for moving forward) is sent to the robot, closing the feedback loop.

