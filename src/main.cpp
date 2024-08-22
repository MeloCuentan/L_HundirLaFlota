#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <driver/i2s.h>
#include <pantallaInicioA.h>
#include <pantallaInicioB.h>
#include <sonidos.h>
#include <EEPROM.h>
#include <Mapa.h>

// Valores de la EEPROM
const uint8_t tamanioEEPROM = 32;
const uint8_t addrEepromVolumen = 0;
const uint8_t addrEepromBrillo = 1;

enum nombreMenu // Creamos nombres para controlar cada menú
{
  MENU_IMAGENES_INICIALES,
  MENU_INICIO,
  MENU_JUEGO,
  MENU_AJUSTES
};
enum nombreSonidos // Creamos nombres para controlar cada sonido
{
  SONIDO_NULO,
  SONIDO_TOCADO,
  SONIDO_AGUA,
  SONIDO_EXPLOSION,
  SONIDO_PULSAR
};
enum nombrePantallas // Creamos el nombre de las pantallas
{
  PANTALLA_ENEMIGOS,
  PANTALLA_ALIADOS,
  AMBAS_PANTALLAS
};
enum nombreBotones // Creamos nombres para controlar cada pulsador
{
  BTN_ARRIBA,
  BTN_IZQUIERDA,
  BTN_ABAJO,
  BTN_DERECHA,
  BTN_CENTRO,
  BTN_DISPARAR,
  BTN_MENU,
  BTN_NULO
};
enum nombreCasillas // Creamos nombres para el control del estado de cada casilla
{
  VACIO,       // No se ha hecho nada en esta casilla
  AGUA,        // Disparada y con resultado agua
  TOCADO,      // Dispadada y con resultado tocado
  FRAGATA,     // Tamaño de dos casillas
  DESTRUCTOR,  // Tamaño de tres casillas
  SUBMARINO,   // Tamaño de tres casillas
  ACORAZADO,   // Tamaño de cuatro casillas
  PORTAAVIONES // Tamaño de cinco casillas
};
enum nombreAjustes // Creamos los nombres para la selección de los distintos ajustes
{
  AJUSTE_NUEVO_JUEGO,
  AJUSTE_VOLUMEN,
  AJUSTE_BRILLO,
  LIMITE_AJUSTE
};

uint8_t nombreMenuActual = MENU_IMAGENES_INICIALES;

// Pines utilizados en I2S
const uint8_t BCK = 16;
const uint8_t DOU = 17;
const uint8_t LRC = 21;
const int8_t DIN = -1;

uint8_t selectorAjuste = 0;

// Variables generales
const uint8_t NUMERO_CUADROS = 8;      // Número de cuadros del mapa en horizontal y vertical
const uint8_t ANCHO_CURSOR = 28;       // Ancho del cuadrado del cursor
const uint8_t ANCHO_CUADRO = 29;       // Ancho de cada cuadro
const uint8_t BRILLO_MINIMO = 30;      // Valor de brillo mínimo que se puede ajustar
bool finJuego = false;                 // Si ha terminado el juego
float volumen = 0.5;                   // Volumen predeterminado
float brillo = 255.0;                  // Brillo predeterminado
int8_t volumenUsuario = volumen * 100; // Valor de volumen ajustado por el usuario
uint8_t alcanzado = 0;                 // Número de impactos logrados
uint8_t brilloUsuario = 100;           // Valor de brillo ajustado por el usuario
uint8_t nombreSonido = SONIDO_NULO;    // Número del sonido a reproducir
int16_t dato = 0;                      // Valor del array de sonido

int16_t audioBuffer[512];
size_t bytesWritten;

// Establecemos la configuración de los pines utilizados
const uint8_t CANTIDAD_BOTONES = 7;                                         // Cantidad de todos los botones utilizados
const uint8_t PIN_BOTONES[CANTIDAD_BOTONES] = {39, 37, 35, 38, 40, 33, 34}; // ARRIBA - IZQUIERDA - ABAJO - DERECHA - CENTRO - DISPARAR - MENU
const uint8_t TFT_LED = 1;                                                  // Pin de conexión del led del TFT
const uint8_t TFT_CS_A = 6;                                                 // Pin CS de la pantalla Aliados
const uint8_t TFT_CS_E = 7;                                                 // Pin CS de la pantalla Enemigos

// Valores de configuracion del PWM
const uint16_t FRECUENCY_PWM = 1000; // Frecencia en Hz
const uint8_t BITS_PWM = 8;          // Número de bits (8Bits = 0 -> 255)
const uint8_t CHANNEL_PWM = 0;       // Canal de PWM

const bool SUBE_BRILLO = true;
const bool BAJA_BRILLO = !SUBE_BRILLO;

bool estadoBoton[CANTIDAD_BOTONES] = {true};         // Estado de los botones
bool estadoBotonAnterior[CANTIDAD_BOTONES] = {true}; // Estado de los botones anterior
uint8_t botonActivado = BTN_NULO;

int8_t cursorEjeX = 0; // Posición X del cursor en el mapa
int8_t cursorEjeY = 0; // Posición Y del cursor en el mapa

uint32_t tiempoRefresco;          // Control de tiempo para refrescar las pantallas
uint32_t intervaloRefersco = 100; // Tiempo de refesco de las pantallas

const uint8_t MAPA_VACIO[NUMERO_CUADROS][NUMERO_CUADROS] = {VACIO}; // Creamos un mapa con todos los valores vacíos

void iniciarEEPROM();        // Inicializar memoria EEPROM
void iniciarPines();         // Inicializa los pines del ESP32S2
void iniciarPantallas();     // Inicializar las pantallas
void iniciarI2S();           // Inicializar I2S
void guardaValoresEEPROM();  // Guarda valores en la EEPROM
void leerValoresEEPROM();    // Lee valores de la EEPROM
void pantallaInicio();       // Mostrar imágenes en pantalla de inicio haciendo un fundido con el brillo
void imagenEstaticaInicio(); // Crear imágen estática donde se quedará durante toda la partida
void controlBrillo(bool tipoBrillo);
void reproduce();      // Reproduce el sonido
void controlBotones(); // Controlar el botón que ha sido pulsado e iniciar la acción correspondiente
void menuActual();
void movimientoCrusorJuego();
void accionBotonesAjustes();
void mostrarPantallaAjustes();
void cambiarVolumen(uint8_t valor);
void cambiarBrillo(uint8_t valor);

TFT_eSPI tft = TFT_eSPI(); // Creamos objeto de una pantalla (se utilizarán las dos cambiando el estado del pin CS)
Mapa enemigos(TFT_CS_E);
Mapa aliados(TFT_CS_A);

void setup()
{
  Serial.begin(115200);

  iniciarEEPROM();
  leerValoresEEPROM();
  iniciarPines();
  iniciarI2S();
  iniciarPantallas();
}

void loop()
{
  controlBotones();
  menuActual();
  reproduce(); // Reproduce el sonido que se haya activado
}

void iniciarEEPROM()
{
  EEPROM.begin(32);
  brilloUsuario = EEPROM.read(addrEepromBrillo);
  volumenUsuario = EEPROM.read(addrEepromVolumen);
  brillo = (float)brilloUsuario * 2.55;
}

void iniciarPines()
{
  ledcAttachPin(TFT_LED, CHANNEL_PWM);
  ledcSetup(CHANNEL_PWM, FRECUENCY_PWM, BITS_PWM);
  ledcWrite(CHANNEL_PWM, 0);

  for (uint8_t i = 0; i < CANTIDAD_BOTONES; i++)
  {
    pinMode(PIN_BOTONES[i], INPUT_PULLUP);
  }
}

void iniciarPantallas()
{
  digitalWrite(TFT_CS_A, LOW);
  digitalWrite(TFT_CS_E, LOW);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  digitalWrite(TFT_CS_A, HIGH);
  digitalWrite(TFT_CS_E, HIGH);
}

void iniciarI2S()
{
  i2s_config_t i2sConfig = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = I2S_MCLK_MULTIPLE_256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0};

  i2s_pin_config_t pinConfig = {
      .bck_io_num = BCK,
      .ws_io_num = LRC,
      .data_out_num = DOU,
      .data_in_num = DIN};

  i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pinConfig);
}

void guardaValoresEEPROM()
{
  EEPROM.write(addrEepromBrillo, brilloUsuario);
  EEPROM.write(addrEepromVolumen, volumenUsuario);
  EEPROM.end();
}

void leerValoresEEPROM()
{
  brilloUsuario = EEPROM.read(addrEepromBrillo);
  volumenUsuario = EEPROM.read(addrEepromVolumen);
  brillo = float(brilloUsuario) * 2.5;
}

void pantallaInicio() // Mostramos una imagen en cada pantalla durante poco tiempo haciendo un fundido de inicio y fin
{
  digitalWrite(TFT_CS_A, LOW);                        // Habilitamos pantalla Aliados
  tft.setSwapBytes(true);                             // Habilitamos la escritura de bytes para la imagen
  tft.pushImage(0, 0, 240, 320, imagenInicioAliados); // Mostramos la imagen de bienvenida
  digitalWrite(TFT_CS_A, HIGH);                       // Deshabilitamos pantalla Aliados

  digitalWrite(TFT_CS_E, LOW);                         // Habilitamos pantalla enemigos
  tft.setSwapBytes(true);                              // Habilitamos la escritura de bytes para la imagen
  tft.pushImage(0, 0, 240, 320, imagenInicioEnemigos); // Mostramos la imagen de bienvenida
  digitalWrite(TFT_CS_E, HIGH);                        // Deshabilitamos pantalla Enemigos

  controlBrillo(SUBE_BRILLO);

  delay(2000); // Hacemos una pausa

  controlBrillo(BAJA_BRILLO);

  digitalWrite(TFT_CS_A, LOW);  // Habilitar pantalla Aliados
  digitalWrite(TFT_CS_E, LOW);  // Habilitar pantalla Enemigos
  tft.fillScreen(TFT_BLACK);    // Fondo Negro
  digitalWrite(TFT_CS_A, HIGH); // Deshabilitamos pantalla Aliados
  digitalWrite(TFT_CS_E, HIGH); // Deshabilitamos pantalla Enemigos
}

void imagenEstaticaInicio() // Creamos toda la imagen que hay que mostrar y dejar de manera etática en ambas pantallas
{
  aliados.dibujarMapa();
  enemigos.dibujarMapa();
  controlBrillo(SUBE_BRILLO);
}

void controlBrillo(bool tipoBrillo)
{
  if (tipoBrillo == SUBE_BRILLO)
  {
    for (uint8_t i = 0; i < uint8_t(brillo); i++) // Creamos un aumento en la iluminación de las pantallas
    {
      ledcWrite(CHANNEL_PWM, i);
      delay(5); // Pequeño retardo
    }
  }
  else if (tipoBrillo == BAJA_BRILLO)
  {
    for (uint8_t i = uint8_t(brillo); i > 0; i--) // Creamos un fundido en la iluminación de las pantallas
    {
      ledcWrite(CHANNEL_PWM, i); // Cambiamos el canal del PWM al valor del bucle
      delay(5);
    }
  }
}

void reproduce() // Reproduce el sonido seleccinoado
{
  static uint8_t sonidoAnterior;
  static uint32_t tamanioBuffer = 0;
  static int32_t i, j;
  float ajusteVolumen = 1.0;

  if (sonidoAnterior != nombreSonido) // Reiniciamos el contador
  {
    sonidoAnterior = nombreSonido;
    dato = 0;
    i = 0;
    j = 0;
  }

  switch (nombreSonido) // Seleccionamos el tipo de sonido
  {
  case SONIDO_NULO:
    break;
  case SONIDO_TOCADO:
    dato = sonidoAgua[i + 1] << 8 | sonidoAgua[i];
    tamanioBuffer = sonidoAguaTam;
    break;
  case SONIDO_AGUA:
    dato = sonidoAgua[i + 1] << 8 | sonidoAgua[i];
    tamanioBuffer = sonidoAguaTam;
    break;
  case SONIDO_EXPLOSION:
    uint8_t valorTemporal;
    dato = sonidoTocado[i + 1] << 8 | sonidoTocado[i];
    tamanioBuffer = sonidoTocadoTam;
    valorTemporal = map(sonidoTocado[i + 1], 0, 255, 0, uint8_t(brillo));
    ledcWrite(CHANNEL_PWM, valorTemporal);
    break;
  case SONIDO_PULSAR:
    ajusteVolumen = 2;
    dato = sonidoPulsar[i + 1] << 8 | sonidoPulsar[i];
    tamanioBuffer = sonidoPulsarTam;
    break;
  }

  audioBuffer[j] = dato * ajusteVolumen * volumen;
  audioBuffer[j + 1] = dato * ajusteVolumen * volumen;

  j += 2;
  i += 2;

  if (j >= sizeof(audioBuffer) / sizeof(int16_t))
  {
    j = 0;
    i2s_write(I2S_NUM_0, audioBuffer, sizeof(audioBuffer), &bytesWritten, portMAX_DELAY);
  }

  if (i >= tamanioBuffer)
  {
    switch (nombreSonido)
    {
    case SONIDO_EXPLOSION:
      nombreSonido = SONIDO_NULO;
      ledcWrite(CHANNEL_PWM, uint8_t(brillo));
      break;
    case SONIDO_AGUA:
      nombreSonido = SONIDO_NULO;
      break;
    case SONIDO_PULSAR:
      nombreSonido = SONIDO_NULO;
      break;
    case SONIDO_TOCADO:
      nombreSonido = SONIDO_EXPLOSION;
      break;
    }
  }
}

void controlBotones() // Controlamos el estado de cualquier pulsador
{
  static uint32_t tiempoDebounce;       // Control del tiempo que irá transcurriendo de antirrebote
  static const uint32_t debounce = 200; // Establecemos un tiempo de antirrebote

  for (uint8_t i = 0; i < CANTIDAD_BOTONES; i++)
  {
    estadoBoton[i] = digitalRead(PIN_BOTONES[i]);
    if (estadoBoton[i] == LOW && estadoBotonAnterior[i] == HIGH)
    {
      estadoBotonAnterior[i] = estadoBoton[i];
      tiempoDebounce = millis();
      botonActivado = i;
    }
  }

  if (millis() - tiempoDebounce >= debounce) // Pasado el tiempo, si se ha pulsado algún botón, reiniciar valor
  {
    for (uint8_t i = 0; i < CANTIDAD_BOTONES; i++)
    {
      if (estadoBotonAnterior[i] == LOW) // Se repite la pulasción pasado el tiempo debounce
        estadoBotonAnterior[i] = HIGH;
    }
  }
}

void menuActual()
{
  switch (nombreMenuActual)
  {
  case MENU_INICIO:
    controlBrillo(SUBE_BRILLO);
    nombreMenuActual = MENU_JUEGO;
    break;
  case MENU_IMAGENES_INICIALES:
    pantallaInicio();
    imagenEstaticaInicio();
    nombreMenuActual = MENU_JUEGO;
    break;
  case MENU_JUEGO:
    movimientoCrusorJuego();
    enemigos.cursorMapa(cursorEjeX, cursorEjeY);
    break;
  case MENU_AJUSTES:
    mostrarPantallaAjustes();
    accionBotonesAjustes();
    break;
  }
}

void movimientoCrusorJuego()
{
  uint8_t estadoActualBoton = botonActivado;
  botonActivado = BTN_NULO;
  switch (estadoActualBoton)
  {
  case BTN_ARRIBA:
    if (cursorEjeY > 0)
    {
      cursorEjeY--;
      if (nombreSonido == SONIDO_NULO)
        nombreSonido = SONIDO_PULSAR;
    }
    break;
  case BTN_ABAJO:
    if (cursorEjeY < 7)
    {
      cursorEjeY++;
      if (nombreSonido == SONIDO_NULO)
        nombreSonido = SONIDO_PULSAR;
    }
    break;
  case BTN_IZQUIERDA:
    if (cursorEjeX > 0)
    {
      cursorEjeX--;
      if (nombreSonido == SONIDO_NULO)
        nombreSonido = SONIDO_PULSAR;
    }
    break;
  case BTN_DERECHA:
    if (cursorEjeX < 7)
    {
      cursorEjeX++;
      if (nombreSonido == SONIDO_NULO)
        nombreSonido = SONIDO_PULSAR;
    }
    break;
  case BTN_CENTRO:
    if (nombreSonido == SONIDO_NULO)
      nombreSonido = SONIDO_PULSAR;
    break;
  case BTN_DISPARAR:
    if (nombreSonido == SONIDO_NULO)
      if (enemigos.disparar(cursorEjeX, cursorEjeY))
        nombreSonido = SONIDO_TOCADO;
      else
        nombreSonido = SONIDO_AGUA;
    break;
  case BTN_MENU:
    if (nombreSonido == SONIDO_NULO)
      nombreSonido = SONIDO_PULSAR;
    nombreMenuActual = MENU_AJUSTES;
    digitalWrite(TFT_CS_A, LOW);
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_A, HIGH);
    break;
  }
}

void accionBotonesAjustes()
{
  uint8_t estadoActualBoton = botonActivado;
  botonActivado = BTN_NULO;
  uint8_t valor = 0;

  switch (estadoActualBoton)
  {
  case BTN_ARRIBA:
    if (selectorAjuste > 0)
    {
      nombreSonido = SONIDO_PULSAR;
      selectorAjuste--;
    }
    break;
  case BTN_ABAJO:
    if (selectorAjuste < LIMITE_AJUSTE - 1)
    {
      nombreSonido = SONIDO_PULSAR;
      selectorAjuste++;
    }
    break;
  case BTN_IZQUIERDA:
    valor = 1;
    break;
  case BTN_DERECHA:
    valor = 2;
    break;
  case BTN_CENTRO:
    break;
  case BTN_DISPARAR:
    if (selectorAjuste == AJUSTE_NUEVO_JUEGO)
    {
      guardaValoresEEPROM();
      nombreMenuActual = MENU_INICIO;
      finJuego = false;
      controlBrillo(BAJA_BRILLO);
      enemigos.vaciarMapa();
      aliados.vaciarMapa();
      aliados.dibujarMapa();
      enemigos.dibujarMapa();
    }
    break;
  case BTN_MENU:
    nombreMenuActual = MENU_JUEGO;
    guardaValoresEEPROM();
    aliados.dibujarMapa();
    delay(500);
    break;
  case BTN_NULO:
    break;
  }

  switch (selectorAjuste)
  {
  case AJUSTE_VOLUMEN:
    cambiarVolumen(valor);
    break;
  case AJUSTE_BRILLO:
    cambiarBrillo(valor);
    break;
  }
}

void cambiarVolumen(uint8_t valor)
{
  switch (valor)
  {
  case 1:
    if (volumenUsuario > 0)
    {
      nombreSonido = SONIDO_PULSAR;
      volumenUsuario -= 10;
      mostrarPantallaAjustes();
    }
    break;
  case 2:
    if (volumenUsuario < 100)
    {
      nombreSonido = SONIDO_PULSAR;
      volumenUsuario += 10;
      mostrarPantallaAjustes();
    }
    break;
  }
}

void cambiarBrillo(uint8_t valor)
{
  switch (valor)
  {
  case 1:
    if (brilloUsuario > BRILLO_MINIMO)
    {
      nombreSonido = SONIDO_PULSAR;
      brilloUsuario -= 10;
      mostrarPantallaAjustes();
    }
    break;
  case 2:
    if (brilloUsuario < 100)
    {
      nombreSonido = SONIDO_PULSAR;
      brilloUsuario += 10;
      mostrarPantallaAjustes();
    }
    if (brilloUsuario > 100)
      brilloUsuario = 100;
    break;
  }
}

void mostrarPantallaAjustes()
{
  static uint32_t tiempoIntermitencia, intervaloIntermitencia = 200;
  static bool intermitencia;

  if (millis() - tiempoIntermitencia >= intervaloIntermitencia)
  {
    tiempoIntermitencia = millis();
    intermitencia = !intermitencia;

    digitalWrite(TFT_CS_A, LOW);

    tft.setTextSize(2);
    tft.setCursor(24, 16);
    tft.print(F("VOLUMEN:"));
    tft.setCursor(24, 32);
    tft.print(F("BRILLO:"));

    // MOSTRAMOS OPCION NUEVO JUEGO
    tft.setCursor(24, 0);
    if (selectorAjuste == AJUSTE_NUEVO_JUEGO)
    {
      if (intermitencia)
      {
        tft.print(F("NUEVO JUEGO"));
      }
      else
      {
        tft.print(F("             "));
      }
    }
    else
    {
      tft.print(F("NUEVO JUEGO"));
    }

    // MOSTRAMOS EL VALOR DE VOLUMEN
    tft.setCursor(120, 16);
    if (selectorAjuste == AJUSTE_VOLUMEN)
    {
      if (intermitencia)
      {
        if (volumenUsuario < 100)
          tft.print(F(" "));
        tft.print(volumenUsuario / 10);
        volumen = float(volumenUsuario) / 100.0;
      }
      else
      {
        tft.print(F("   "));
      }
    }
    else
    {
      if (volumenUsuario < 100)
        tft.print(F(" "));
      tft.print(volumenUsuario / 10);
      volumen = float(volumenUsuario) / 100.0;
    }

    // MOSTRAMOS EL VALOR DE BRILLO
    tft.setCursor(120, 32);
    if (selectorAjuste == AJUSTE_BRILLO)
    {
      if (intermitencia)
      {
        if (brilloUsuario < 100)
          tft.print(F(" "));
        tft.print(brilloUsuario / 10);
      }
      else
      {
        tft.print(F("   "));
      }
    }
    else
    {
      if (brilloUsuario < 100)
        tft.print(F(" "));
      tft.print(brilloUsuario / 10);
    }
    brillo = float(brilloUsuario * 255) / 100.0;
    ledcWrite(CHANNEL_PWM, uint8_t(brillo));

    digitalWrite(TFT_CS_A, HIGH);
  }
}