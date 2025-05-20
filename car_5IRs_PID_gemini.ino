// === Definición de Pines ===
// Sensores IR (Analógicos) - Ajustar según Tabla 2
const int sensorPins[] = {A0, A1, A2, A3, A4}; // S1, S2, S3, S4, S5
const int NUM_SENSORS = 5;

// Controlador L298N (Motores) - Ajustar según Tabla 3
const int IN1_L = 22; // Motor Izquierdo
const int IN2_L = 23;
const int ENA_L = 2;  // PWM para velocidad motor izquierdo
const int IN3_R = 24; // Motor Derecho
const int IN4_R = 25;
const int ENB_R = 3;  // PWM para velocidad motor derecho

// === Parámetros de Calibración de Sensores ===
int sensorMin [NUM_SENSORS]; // Lectura mínima (blanco)
int sensorMax [NUM_SENSORS]; // Lectura máxima (negro)
// Para la configuración de línea ancha, S3 (sensorPins) debe estar en negro.
// S1 (sensorPins) y S5 (sensorPins) en blanco.
// S2 (sensorPins) y S4 (sensorPins) en los bordes (gris).

// === Variables para Histéresis de Detección de Línea ===
int lineLostCounter = 0;
const int LINE_LOST_THRESHOLD_COUNT = 20; // Considerar línea perdida después de N lecturas consecutivas sin detección

// === Parámetros PID ===
float Kp = 0.035;  // Ganancia Proporcional (AJUSTAR) -  TO_DO LaMejor config puede que haya que bajar o subir un poco 
float Ki = 0.005; // Ganancia Integral (AJUSTAR)
float Kd = 0.00005;  // Ganancia Derivativa (AJUSTAR)

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// === Parámetros de Velocidad de Motores ===
int baseSpeed = 78; // Velocidad base de los motores (0-255) (AJUSTAR)
int motorSpeedL = 0;
int motorSpeedR = 0;

// === Constantes para el cálculo de posición ===
// El setpoint ideal es cuando la línea está perfectamente centrada bajo S3.
// Con 5 sensores, podemos asignar valores de 0 (extrema izq) a 4000 (extrema der).
// El centro (S3) sería 2000.
int setpoint = 2000; 

// === Configuración Inicial ===
void setup() {
  Serial.begin(9600); // Para depuración

  // Configurar pines de motores como SALIDA
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(ENA_L, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);
  pinMode(ENB_R, OUTPUT);

  // (Opcional) Configurar pines de sensores como ENTRADA (no estrictamente necesario para analogRead)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Calibrar Sensores
  //calibrateSensors();
  default_calibrateSensors();
  
  Serial.println("Calibracion completa");
  // Bucle infinito para depurar
  while (false) 
  {
    delay(1000); // Pausa antes de empezar
    Serial.println(" BLOCKED ");

    /*/
    int position = readLinePosition();
    Serial.print(" readLinePosition(): ");
    Serial.println(position);
    // TO_DO TO_DEBUG 22:22:21.776 ->  readLinePosition(): 2333
    // TO_DO TO_DEBUG 22:22:21.776 ->  readLinePosition(): 1666
    // TO_DO TO_DEBUG 22:22:21.776 ->  readLinePosition(): 3500
    */
    Serial.println("");
    int sensorValues[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = 1023 - analogRead(sensorPins[i]);
    }
    Serial.print("analogRead sensorValues: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
    }
    
    Serial.println("");
    for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = !digitalRead(sensorPins[i]) * 900;
    }
    Serial.print("digitalRead sensorValues: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
    }
  }
  //delay(1000); // Pausa antes de empezar

  Serial.println("LED Blinking...");
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i<5; i++)
  { 
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(250);
  }
  for (int i = 0; i<3; i++)
  { 
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(500);
  }
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

  Serial.println("=> Iniciando seguimiento...");

}

// === Bucle Principal ===
void loop() {
  // 1. Leer sensores y calcular la posición de la línea
  int position = readLinePosition();

  // 2. Calcular el error PID
  // Si readLinePosition devuelve -1 (línea perdida), manejarlo.
  if (position == -1) { // Línea perdida
    // Acción para línea perdida (ej. parar, buscar, usar último error conocido)
    // Aquí, simplemente paramos los motores y reiniciamos el PID.
    setMotorControl(0, 0); 
    lastError = 0; // O mantener el último error para intentar girar hacia él
    integral = 0;
    Serial.println("Linea perdida!");
    boolean ledState = LOW; // Initial LED state (off)
    while (true)
    {
      //Serial.println(" LOOP - Linea perdida! - STOP");
      ledState = !ledState; // Toggle LED state
      digitalWrite(LED_BUILTIN, ledState);
      delay(150);
    }
    // Podría implementarse una estrategia de búsqueda aquí.
    // Por ahora, si se pierde la línea, el error será grande si position = 0 o 4000
    // y el setpoint es 2000. Esto hará que el robot gire bruscamente.
    // Una mejor aproximación sería usar el último error conocido con signo.
  } else {
    error = position - setpoint; // 'setpoint' es el valor deseado para la posición (ej. 2000 para el centro)

    // 3. Calcular términos PID
    integral += error;
    // Anti-windup para el término integral (limitar su valor)
    if (integral > 3000) integral = 3000;
    if (integral < -3000) integral = -3000;

    derivative = error - lastError;
    lastError = error;

    //pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative); // TO_DEBUG con Error 0 se queda el PID en 15 producto de la integral
    pidOutput = (Kp * error);


    // 4. Calcular velocidad de los motores
    motorSpeedL = baseSpeed + pidOutput;
    motorSpeedR = baseSpeed - pidOutput;

    // Limitar velocidades al rango 0-255
    motorSpeedL = constrain(motorSpeedL, 0, 255);
    motorSpeedR = constrain(motorSpeedR, 0, 255);

    // 5. Controlar los motores
    setMotorControl(motorSpeedL, motorSpeedR);
  }
  
  // Imprimir valores para depuración (opcional, puede ralentizar el bucle)
  /*
  Serial.print(" (*) Pos: "); Serial.print(position);
  Serial.print(" Err: "); Serial.print(error);
  Serial.print(" PID: "); Serial.print(pidOutput);
  Serial.print(" L: "); Serial.print(motorSpeedL);
  Serial.print(" R: "); Serial.println(motorSpeedR);
  */
  delay(10); // Pequeño delay para estabilidad del bucle, ajustar o eliminar para max velocidad
  
  //Serial.println(" *** SACAR LONG DELAY *** "");
  //delay(2000); // TO_Debug
}

// === Función de Calibración de Sensores ===
void calibrateSensors() {
  Serial.println("Iniciando calibracion de sensores...");
  pinMode(13, OUTPUT); // Usar LED integrado para indicar calibración

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023; // Inicializar con el valor máximo posible
    sensorMax[i] = 0;    // Inicializar con el valor mínimo posible
  }

  // Parpadear LED para indicar inicio de calibración
  for (int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH); delay(50);
    digitalWrite(13, LOW); delay(50);
  }
  Serial.println("Mueva los sensores sobre la linea negra y la superficie blanca durante 10 segundos.");

  long calibrationStartTime = millis();
  while (millis() - calibrationStartTime < 15000) { // Calibrar por 15 segundos
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    // Pequeño parpadeo para mostrar que está calibrando
    digitalWrite(13,!digitalRead(13));
    delay(20); 
  }
  digitalWrite(13, LOW); // Apagar LED al finalizar

  // Imprimir valores de calibración
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor "); Serial.print(i);
    Serial.print(" Min: "); Serial.print(sensorMin[i]);
    Serial.print(" Max: "); Serial.println(sensorMax[i]);
  }
  // Validar que min no sea igual a max (evitar división por cero)
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMin[i] >= sensorMax[i]) {
        Serial.print("Error de calibracion en sensor "); Serial.println(i);
        // Asignar valores por defecto o manejar el error
        sensorMin[i] = 200; // Valor típico para blanco
        sensorMax[i] = 800; // Valor típico para negro
    }
  }
}

void default_calibrateSensors() {
  Serial.println("DEFAULT Iniciando calibracion de sensores...");
  pinMode(13, OUTPUT); // Usar LED integrado para indicar calibración

  // Validar que min no sea igual a max (evitar división por cero)
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMin[i] >= sensorMax[i]) {
        Serial.print("Error de calibracion en sensor "); Serial.println(i);
        // Asignar valores por defecto o manejar el error
        sensorMin[i] = 200; // Valor típico para blanco
        sensorMax[i] = 800; // Valor típico para negro
    }
  }
  // Imprimir valores de calibración
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("DEFAULT Sensor "); Serial.print(i);
    Serial.print(" Min: "); Serial.print(sensorMin[i]);
    Serial.print(" Max: "); Serial.println(sensorMax[i]);
  }
}
// === Función para Leer Sensores y Determinar Posición de la Línea ===
// Devuelve un valor entre 0 y (NUM_SENSORS-1)*1000. Ej: 0-4000 para 5 sensores.
// 2000 sería el centro (línea bajo el sensor S3).
// Devuelve -1 si la línea no es detectada.
int OLD_readLinePosition() {
  unsigned int sensorValues[5];
  bool onLine = false; // Flag para saber si algún sensor detecta la línea
  unsigned long avg = 0; // Usado para el promedio ponderado
  unsigned int sum = 0;  // Usado para la suma de las lecturas (denominador)

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Leer valor crudo
    //int val = analogRead(sensorPins[i]);
    //int val = 1023 - analogRead(sensorPins[i]);
    int val = !digitalRead(sensorPins[i]);
    
    // Normalizar el valor del sensor entre 0 y 1000
    // 0 = blanco perfecto, 1000 = negro perfecto (según calibración)
    // Se invierte la lógica si min > max (sensores que dan alto para blanco)
    // Asumimos min = blanco, max = negro
    int calibratedValue = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    calibratedValue = constrain(calibratedValue, 0, 1000); // Limitar al rango
    sensorValues[i] = calibratedValue;

    if (sensorValues[i] > 800) { // Considerar "en línea" si está por encima de un umbral (ej. 20% de negro)
      onLine = true;
    }
    avg += (unsigned long)sensorValues[i] * (i * 1000); // Promedio ponderado
    sum += sensorValues[i];
  }

  Serial.print("sensorValues: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.print(" (#) onLine: "); Serial.print(onLine);
  Serial.print(" avg: "); Serial.print(avg);
  Serial.print(" sum: "); Serial.print(sum);
  Serial.print(" lastError: "); Serial.println(lastError);

  if ((!onLine) || (sum == 0)) { // Si no hay línea detectada o suma es cero
    // Devolver último error conocido o un valor que indique búsqueda
    // Si el último error era < 0, la línea estaba a la izquierda, devolver 0 (extrema izquierda)
    // Si el último error era > 0, la línea estaba a la derecha, devolver 4000 (extrema derecha)
    if (lastError < -100) return 0; // Estaba a la izquierda, seguir buscando a la izquierda
    else if (lastError > 100) return (NUM_SENSORS - 1) * 1000; // Estaba a la derecha
    return -1; // Indicador de línea perdida general
  }

  return avg / sum;
}


// === Función para Leer Sensores y Determinar Posición de la Línea ===
int readLinePosition() {
  unsigned int sensor_values_calibrated[NUM_SENSORS];
  bool onLineIndividual[NUM_SENSORS] = {false}; // Para saber si cada sensor ve "algo de línea"
  bool onLineOverall = false; // Flag para saber si al menos un sensor detecta la línea claramente
  
  unsigned long avg = 0; // Numerador para el promedio ponderado
  unsigned int sum = 0;  // Denominador para el promedio ponderado

  // Leer y calibrar cada sensor
  //Serial.println("Leer y calibrar cada sensor.");
  for (int i = 0; i < NUM_SENSORS; i++) {
    //int rawValue = 1023 - analogRead(sensorPins[i]); // Lectura directa
    int rawValue = !digitalRead(sensorPins[i]) * 900;
    //Serial.println("Lectura directa DONE.");


    // Normalizar el valor del sensor entre 0 y 1000
    // Ahora, 0 = blanco perfecto (sensorMin), 1000 = negro perfecto (sensorMax)
    // Los valores de sensorMin y sensorMax se obtienen de la calibración real.
    int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], 0, 1000);
    calibratedValue = constrain(calibratedValue, 0, 1000);
    sensor_values_calibrated[i] = calibratedValue;

    // Considerar que un sensor está "en la línea" o "viendo algo de la línea"
    // si su valor calibrado supera un umbral. Este umbral es clave.
    // Si 200 significa "20% negro", ajústalo según tus pruebas.
    // Un valor más bajo (ej. 100-150) podría ser mejor para tu caso de línea estrecha/bordes.
    if (sensor_values_calibrated[i] > 800) { // Umbral para considerar "activo" o "viendo línea/borde"
      onLineIndividual[i] = true;
      onLineOverall = true; // Si al menos uno está activo, consideramos que estamos "en general" sobre la línea
    }
  }

  // Implementación de Histéresis
  if (onLineOverall) {
    lineLostCounter = 0; // Si vemos la línea, reseteamos el contador
  } else {
    lineLostCounter++;   // Si no vemos la línea, incrementamos
  }

  Serial.print(" onLineOverall: "); Serial.print(onLineOverall);
  Serial.print(" lineLostCounter: "); Serial.print(lineLostCounter);
  // Depuración de sensores
  Serial.print(" Calibrados: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensor_values_calibrated[i]); Serial.print(" ");
  }
  //Serial.print(" onLineOverall: "); Serial.print(onLineOverall);
  //Serial.print(" lineLostCounter: "); Serial.print(lineLostCounter);
  // Fin depuración

  // Si el contador de línea perdida supera el umbral, o si la suma es cero (nunca debería pasar si onLineOverall es true)
  if (lineLostCounter >= LINE_LOST_THRESHOLD_COUNT) {
    Serial.print(" -> PERDIDA (hys)");
    // Decidir qué valor devolver para indicar "línea perdida y buscar"
    // Usar lastError para decidir hacia dónde buscar es una buena estrategia.
    // Un error grande y negativo significa que la línea estaba a la izquierda.
    // Un error grande y positivo significa que la línea estaba a la derecha.
    if (lastError < -100) { // Ajustar este -100 si es necesario
      Serial.println(" -> Buscar Izq");
      return 0; // Valor que representa extrema izquierda
    } else if (lastError > 100) { // Ajustar este 100 si es necesario
      Serial.println(" -> Buscar Der");
      return (NUM_SENSORS - 1) * 1000; // Valor que representa extrema derecha
    } else {
      Serial.println(" -> Perdida, sin dir clara");
      return -1; // Indicador genérico de línea perdida, PID se detendrá.
    }
  }
  
  // Si no se considera perdida por histéresis, pero onLineOverall sigue siendo false
  // (porque lineLostCounter < LINE_LOST_THRESHOLD_COUNT), aún podríamos estar en una transición.
  // Si sum fuera cero aquí, sería problemático.
  // El cálculo del promedio ponderado solo tiene sentido si hay algo que sumar.
  if (!onLineOverall && lineLostCounter < LINE_LOST_THRESHOLD_COUNT) {
      // Estamos en transición, no hemos perdido la línea "oficialmente" aún por histéresis.
      // Podríamos devolver la última posición conocida si la tuviéramos, o
      // intentar un cálculo con los valores bajos actuales si sum no es cero.
      // O, más simple, si no hay nada "claramente" en línea, pero aún no es "perdida",
      // podríamos devolver la última posición conocida del error.
      // Por ahora, si onLineOverall es false, pero no hemos alcanzado el conteo de perdida,
      // actuaremos como si estuviera perdida para forzar una busqueda basada en lastError
      // o devolver -1 para que el PID se detenga y no use valores de avg/sum basados en ruido.
      // Esto depende de la agresividad que quieras en la recuperación.
      // Para simplificar, si onLineOverall es FALSO (ningún sensor supera el umbral de 150),
      // incluso si el contador de histéresis no ha llegado al límite,
      // es mejor no calcular avg/sum si 'sum' va a ser muy bajo o cero.
      // La lógica de arriba (lineLostCounter >= ...) ya cubre esto.
      // Si llegamos aquí, es porque onLineOverall es TRUE, o lineLostCounter < THRESHOLD_COUNT.
      // Si onLineOverall es TRUE, entonces sum no debería ser 0.
  }


  // Calcular promedio ponderado usando los valores calibrados
  // Solo si al menos un sensor detectó algo (onLineOverall = true)
  // y no hemos superado el contador de línea perdida por histéresis.
  if (onLineOverall) { // Esta condición es redundante si la lógica de arriba ya devuelve en caso de perdida
                      // pero la dejamos por claridad.
    for (int i = 0; i < NUM_SENSORS; i++) {
      // Usamos sensor_values_calibrated[i] directamente. Si es bajo (casi blanco),
      // contribuirá poco a la suma y al promedio ponderado.
      // Si es alto (casi negro), contribuirá mucho.
      avg += (unsigned long)sensor_values_calibrated[i] * (i * 1000);
      sum += sensor_values_calibrated[i];
    }

    if (sum == 0) {
      // Esto no debería ocurrir si onLineOverall es true y el umbral > 0,
      // pero como salvaguarda:
      Serial.print(" -> SUMA CERO con onLineOverall TRUE!"); // Error de lógica o umbral muy bajo
      if (lastError < -100) return 0;
      else if (lastError > 100) return (NUM_SENSORS - 1) * 1000;
      return -1;
    }
    Serial.print(" -> avg/sum: "); Serial.print(avg/sum);
    Serial.println();
    return avg / sum;
  } else {
    // Si onLineOverall es false, pero lineLostCounter aún no ha llegado al umbral
    // (ej. es la 1ra o 2da vez que no se ve línea).
    // Podríamos devolver la última posición conocida (requiere variable estática),
    // o la indicación de búsqueda basada en lastError.
    // La lógica del contador ya maneja esto más arriba. Si se llega aquí, algo está mal.
    // Por seguridad, si no hay línea y no se ha activado la pérdida por histéresis aun:
    Serial.println(" -> En transicion, usando lastError para dir.");
    //if (lastError < -100) return 0; 
    //else if (lastError > 100) return (NUM_SENSORS - 1) * 1000;
    return setpoint; // O devolver el setpoint para que intente ir recto si no hay info
  }
}


// === Función para Controlar los Motores ===
void setMotorControl(int speedL, int speedR) {
  // Control Motor Izquierdo
  if (speedL >= 0) {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(ENA_L, speedL);
  } else { // Para ir hacia atrás, invertir lógica y usar abs(speedL)
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    analogWrite(ENA_L, -speedL); // speedL es negativo
  }

  // Control Motor Derecho
  if (speedR >= 0) {
    digitalWrite(IN3_R, HIGH);
    digitalWrite(IN4_R, LOW);
    analogWrite(ENB_R, speedR);
  } else { // Para ir hacia atrás
    digitalWrite(IN3_R, LOW);
    digitalWrite(IN4_R, HIGH);
    analogWrite(ENB_R, -speedR); // speedR es negativo
  }
}
