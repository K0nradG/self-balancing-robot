# Identyfikacja i Sterowanie LQR dla Dwukołowego Robota Balansującego

Ten projekt zawiera kompletny zbiór narzędzi do identyfikacji modelu matematycznego dwukołowego robota balansującego na podstawie rzeczywistych danych z czujników, a następnie zaprojektowania dla niego optymalnego regulatora LQR.

Ze względu na to, że robot w pozycji stojącej jest układem wysoce niestabilnym, **identyfikacja przeprowadzana jest w bezpiecznej pozycji wiszącej**, a wyznaczony model jest następnie transformowany matematycznie do pozycji odwróconej.

## 1. Model Matematyczny

Układ opisujemy za pomocą równań stanu w dziedzinie czasu ciągłego:

$$\dot{x} = A_c x + B_c u$$

Gdzie wektor stanu $x$ składa się z 4 zmiennych:

- $x_1$ - Pozycja wózka/robota $[m]$
- $x_2$ - Prędkość liniowa robota $[m/s]$
- $x_3$ - Kąt pochylenia względem pionu $[rad]$
- $x_4$ - Prędkość kątowa pochylenia $[rad/s]$

Sygnał sterujący $u$ to w naszym przypadku wypełnienie sygnału PWM podawanego na silniki (w procentach $[-100, 100]$). Dzięki identyfikacji na rzeczywistych danych, "czarna skrzynka" w postaci dynamiki silników DC i przekładni jest automatycznie zaszyta w wyliczonej macierzy wejść $B_c$.

------

## 2. Zbieranie Danych i Identyfikacja (Wahadło Wiszące)

Aby zebrać poprawne dane (bez wpływu nieliniowości wynikających z gwałtownych upadków robota), robot jest podwieszany "do góry nogami" (jako stabilne wahadło fizyczne). Na silniki podawany jest sygnał sterujący , a my rejestrujemy odpowiedź układu: pozycję, prędkość, kąt i prędkość kątową w stałych odstępach czasu $dt$.

### Jak działa Metoda Najmniejszych Kwadratów (OLS) w tym przypadku?

Naszym celem jest znalezienie takich macierzy dyskretnych $A_d$ i $B_d$, które najlepiej opisują ewolucję układu krok po kroku, zgodnie z równaniem różnicowym:

$$x_{k+1} = A_d x_k + B_d u_k$$

Ponieważ zebraliśmy tysiące próbek, możemy ułożyć z nich ogromny układ równań. Układamy nasze dane w macierze:

- **$X_{k+1}$**: Macierz przyszłych stanów (od próbki 2 do końca).
- **$X_k$**: Macierz obecnych stanów (od próbki 1 do przedostatniej).
- **$U_k$**: Macierz sygnałów sterujących (od próbki 1 do przedostatniej).

Możemy zapisać równanie dla wszystkich próbek naraz w postaci macierzowej:

$$X_{k+1} = [X_k \quad U_k] \begin{bmatrix} A_d^T \\ B_d^T \end{bmatrix}$$

Zdefiniujmy macierz regresorów $\Phi$ oraz poszukiwaną macierz parametrów $\Theta$:

- $\Phi = [X_k \quad U_k]$ (macierz zawierająca to, co "wiemy" na danym kroku).
- $\Theta = \begin{bmatrix} A_d^T \\ B_d^T \end{bmatrix}$ (macierz współczynników, których szukamy).

Równanie upraszcza się do:

$$X_{k+1} = \Phi \Theta$$

Ponieważ w rzeczywistości występują szumy pomiarowe, to równanie nigdy nie będzie spełnione idealnie. Dlatego korzystamy z **Metody Najmniejszych Kwadratów (Ordinary Least Squares)**, która szuka takiej macierzy $\Theta$, która minimalizuje sumę kwadratów błędów predykcji. Rozwiązanie analityczne tego problemu (wykorzystujące pseudoodwrotność Moore'a-Penrose'a) to:

$$\Theta = (\Phi^T \Phi)^{-1} \Phi^T X_{k+1}$$

Po wyliczeniu $\Theta$, transponujemy ją i "odcinamy" odpowiednie fragmenty, aby otrzymać nasze macierze $A_d$ (pierwsze 4 kolumny) oraz $B_d$ (ostatnia kolumna).

### Konwersja na model ciągły

Macierze $A_d$ i $B_d$ są ściśle związane z naszym czasem próbkowania $dt$. Aby model był użyteczny niezależnie od szybkości pętli mikrokontrolera i aby odzwierciedlał czystą fizykę układu, konwertujemy go do dziedziny czasu ciągłego za pomocą logarytmu macierzowego:

$$A_c = \frac{\log_m(A_d)}{dt}$$

$$B_c = (A_d - I)^{-1} A_c B_d$$

------

## 3. Inwersja Modelu (Przejście do pozycji stojącej)

Zidentyfikowana macierz $A_c$ opisuje wahadło **wiszące** (stabilne). Aby użyć jej do zaprojektowania regulatora dla robota **stojącego**, musimy uwzględnić zmianę kierunku działania grawitacji w równaniach dynamiki układu.

Wymaga to odwrócenia znaków w dwóch kluczowych miejscach macierzy $A_c$:

1. Współczynnik $A_c[3, 2]$ (wpływ grawitacji na przyspieszenie kątowe). Dla wahadła wiszącego jest ujemny (ściąga do zera). Zmieniamy go na **dodatni**, co tworzy biegun niestabilny (grawitacja przewraca robota odpychając go od pionu).
2. Współczynnik $A_c[1, 2]$ (wpływ kąta na przyspieszenie kół) również zmienia znak na **przeciwny** ze względu na kinematykę odwróconego wahadła.

Python

```
A_upright = A_c.copy()
A_upright[3, 2] = abs(A_c[3, 2]) # Grawitacja staje się siłą destabilizującą
A_upright[1, 2] = -A_c[1, 2]     # Odwrócenie wpływu sprzężenia na koła
```

------

## 4. Projektowanie Regulatora LQR

Mając matematyczny model układu niestabilnego ($A_{upright}, B_c$), projektujemy Liniowo-Kwadratowy Regulator (LQR), który generuje optymalne sterowanie $u = -Kx$, minimalizując funkcję kosztu na podstawie zdefiniowanych macierzy wag:

- **Macierz Q (Kary za błąd stanu):** Wektor diagonalny odpowiadający za stany $[x, \dot{x}, \theta, \dot{\theta}]$. Wyższe wartości zmuszają system do szybszego niwelowania uchybu na danym stanie (zazwyczaj najwyższa waga przypisywana jest do kąta $\theta$).
- **Macierz R (Koszt sterowania):** Skalar lub macierz karząca za zużycie energii. Mniejsza wartość $R$ oznacza, że regulator może używać bardziej "agresywnych" sygnałów PWM, nie przejmując się zużyciem prądu.

Rozwiązując ciągłe algebraiczne równanie Riccatiego (ARE) dla podanych $Q$ i $R$, system wylicza wektor wzmocnień $K$, który służy bezpośrednio do wyliczania sygnału PWM w pętli sprzężenia zwrotnego.



-----------------------------------------------------------------------------------------------------------------------------

# Identification and LQR Control for a Two-Wheeled Balancing Robot

This project provides a complete toolset for identifying the mathematical model of a Two-Wheeled Inverted Pendulum (TWIP) robot based on real sensor data, and subsequently designing an optimal LQR controller for it.

Because the robot in an upright position is a highly unstable system, **the identification process is performed in a safe, hanging position**. The identified model is then mathematically transformed to represent the inverted (upright) position.

## 1. Mathematical Model

The system is described using continuous-time state-space equations:

$$\dot{x} = A_c x + B_c u$$

Where the state vector $x$ consists of 4 variables:

- $x_1$ - Robot position $[m]$
- $x_2$ - Linear velocity $[m/s]$
- $x_3$ - Pitch angle relative to the vertical $[rad]$
- $x_4$ - Pitch angular velocity $[rad/s]$

In our case, the control signal $u$ is the PWM duty cycle applied to the motors (as a percentage $[-100, 100]$). Thanks to data-driven identification, the "black box" dynamics of the DC motors and gearboxes are automatically embedded into the computed input matrix $B_c$.

------

## 2. Data Collection and Identification (Hanging Pendulum)

To collect clean data (without the nonlinearities caused by the robot violently falling over), the robot is suspended "upside down" (acting as a stable physical pendulum). A random or pseudo-random control signal (e.g., PRBS noise) is applied to the motors, and we record the system's response: position, velocity, angle, and angular velocity at fixed time intervals $dt$.

### How does Ordinary Least Squares (OLS) work here?

Our goal is to find the discrete matrices $A_d$ and $B_d$ that best describe the step-by-step evolution of the system, according to the difference equation:

$$x_{k+1} = A_d x_k + B_d u_k$$

Since we have collected thousands of samples, we can form a massive system of equations. We arrange our data into matrices:

- **$X_{k+1}$**: Matrix of future states (from sample 2 to the end).
- **$X_k$**: Matrix of current states (from sample 1 to the second-to-last).
- **$U_k$**: Matrix of control signals (from sample 1 to the second-to-last).

We can write the equation for all samples at once in matrix form:

$$X_{k+1} = [X_k \quad U_k] \begin{bmatrix} A_d^T \\ B_d^T \end{bmatrix}$$

Let's define the regressor matrix $\Phi$ and the target parameter matrix $\Theta$:

- $\Phi = [X_k \quad U_k]$ (the matrix containing what we "know" at a given step).
- $\Theta = \begin{bmatrix} A_d^T \\ B_d^T \end{bmatrix}$ (the matrix of coefficients we are looking for).

The equation simplifies to:

$$X_{k+1} = \Phi \Theta$$

Because of real-world measurement noise, this equation will never be perfectly satisfied. Therefore, we use the **Ordinary Least Squares (OLS)** method, which finds the matrix $\Theta$ that minimizes the sum of squared prediction errors. The analytical solution to this problem (using the Moore-Penrose pseudoinverse) is:

$$\Theta = (\Phi^T \Phi)^{-1} \Phi^T X_{k+1}$$

After calculating $\Theta$, we transpose it and "slice" it into appropriate parts to extract our $A_d$ matrix (first 4 columns) and $B_d$ matrix (last column).

### Conversion to a Continuous Model

The matrices $A_d$ and $B_d$ are strictly tied to our sampling time $dt$. For the model to be universally useful regardless of the microcontroller's loop speed, and to reflect the pure physics of the system, we convert it to the continuous-time domain using the matrix logarithm:

$$A_c = \frac{\log_m(A_d)}{dt}$$

$$B_c = (A_d - I)^{-1} A_c B_d$$

------

## 3. Model Inversion (Transition to Upright Position)

The identified matrix $A_c$ describes a **hanging** (stable) pendulum. To use it for designing a controller for the **standing** robot, we must account for the change in gravity's direction in the system dynamics.

This requires flipping the signs in two key locations of the $A_c$ matrix:

1. Coefficient $A_c[3, 2]$ (the effect of gravity on angular acceleration). For a hanging pendulum, this is negative (pulling the angle back to zero). We change it to **positive**, creating an unstable pole (gravity now pushes the robot away from the vertical).
2. Coefficient $A_c[1, 2]$ (the effect of the angle on wheel acceleration) also flips its sign due to the kinematics of the inverted pendulum.

Python

```
A_upright = A_c.copy()
A_upright[3, 2] = abs(A_c[3, 2]) # Gravity becomes a destabilizing force
A_upright[1, 2] = -A_c[1, 2]     # Invert the coupling effect on the wheels
```

------

## 4. LQR Control Design

Having the mathematical model of the unstable system ($A_{upright}, B_c$), we design a Linear-Quadratic Regulator (LQR) that generates the optimal control law $u = -Kx$ by minimizing a cost function based on defined weight matrices:

- **Q Matrix (State Error Penalty):** A diagonal matrix corresponding to the states $[x, \dot{x}, \theta, \dot{\theta}]$. Higher values force the system to eliminate the error in that specific state much faster (typically, the highest weight is assigned to the angle $\theta$).
- **R Matrix (Control Effort Penalty):** A scalar or matrix penalizing energy usage. A smaller $R$ value means the controller is allowed to use more "aggressive" PWM signals without worrying about current draw.

By solving the Continuous Algebraic Riccati Equation (CARE) for the given $Q$ and $R$, the system computes the gain vector $K$, which is directly used to calculate the PWM signal in the feedback loop.

------

