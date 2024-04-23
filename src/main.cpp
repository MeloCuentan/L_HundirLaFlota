/*
    Juego de Hundir La Flota

    Este juego tendrá dos pantallas TFT de 2.4" con una resolución de 240 x 320
    En una pantalla mostrará el mapa de los enemigos y en la otra el de los aliados
    Tendrá la opción de añadir sonido para simular el disparo, y en caso de ser "TOCADO" que haga una pequeña explosión
    A parte de lo anterior, se puede simular al mismo tiempo que la explosión, que cambie de manera repentina el brillo del mapa que sea el afectdo

    Como controles, tendremos un mando de 4 posiciones (ARRIBA, ABAJO, IZQUIERDA, DERECHA) que también tiene un pin CENTRAL y dos botones añadidos que
  serán los de DISPARO y MENU

    El orden del menú será el siguiente:

            |  LOCAL --------> | COLOCAR BARCOS
            |                  | COMIENZA EL JUEGO
    MENU -> |
            |  OTRO JUGADOR -> | BUSCAR DISPOSITIVO
            |                  | COLOCAR BARCOS EN AMBOS DISPOSITIVOS
            |                  | COMIENZA EL JUEGO

    Para movernos por el menú, se utilizará el mando de cuatro posiciones, la selección será el botón DISPARAR y para volver a atrás será el botón MENU

    La dinámica del juego será la siguiente:
    Cuando cada jugador termine de colocar sus barcos, se generará un número aleatorio el cual enviará al otro dispositivo para que empiece el mayor.
    El jugador se podrá mover por el mapa con el mando para seleccionar donde quiere efecturar el disparo y luego pulsar el botón DISPARAR.
    En la pantalla aparecerá ese recuadro en AZUL si ha sido agua, o ROJO si ha sigo TOCADO y cambiará el turno al siguiente jugador.
    Este mecánica se irá repitiendo hasta que se termine con el úntimo barco.
    Cuando esto ocurra, en la pantalla del ganador, aparecerá YOU WIN, y en el perdedor pondrá YOU LOST ocupando las dos pantallas en ambos casos.
    En este punto solo se podrá acceder al MENU para volver a seleccionar el modo de juego e iniciar una nueva partida
    Para la colocación de los barcos, irán apareciendo primero el PORTAAVIONES, ocupando directamente las 5 casillas. El usuario con el mando, podrá moverlo
  por el mapa para colocarlo en la posición que prefiera dándole al botón DISPARO. El siguiente barco en colocar será el acorazado que ocupa 4 casillas. Moviendo
  el mando podrá colocarlo en la posición que prefiera siempre y cuando no coincida ninguna parte con el barco anterior, y así con el resto de los barcos.
    Para girar el barco, hay que pulsar el botón CENTRO.
    Una vez terminado con todos los barcos, aparecerán dos opciones, que serán para modificar o para comenzar. Si le damos a modificar, podemos movernos por el
  mapa y al estar encima de uno de nuestros barcos, podremos darle al botón DISPARAR para seleccionar el barco y moverlo a la posición que queramos, repitiendo
  los pasos anteriores.

*/

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <driver/i2s.h>
#include <pantallaInicioA.h>
#include <pantallaInicioB.h>
#include <sonidos.h>

// Pines utilizados en Wemos S2 MINI
const uint8_t BCK = 16;
const uint8_t DOU = 17;
const uint8_t LRC = 21;
const int8_t DIN = -1;

enum nombreSonidos
{
  SONIDO_NULO,
  SONIDO_TOCADO,
  SONIDO_AGUA,
  SONIDO_EXPLOSION,
  SONIDO_PULSAR
};

// Variables generales
uint8_t alcanzado = 0;
bool finJuego = false;
float volumen = 0.5; // Volumen predeterminado
uint8_t sonido = 0;   // Indice de sonido
int j = 0;            // conteo del array
int i = 0;            // conteo del array
int16_t dato = 0;     // Valor del array
uint32_t tamanioBuffer = 0;
i2s_config_t i2sConfig;

int16_t audioBuffer[512];
size_t bytesWritten;

// Establecemos la configuración de los pines utilizados
const uint8_t cantidadBotones = 7;                                        // Cantidad de todos los botones utilizados
const uint8_t pinBotones[cantidadBotones] = {39, 37, 35, 38, 40, 33, 34}; // ARRIBA - IZQUIERDA - ABAJO - DERECHA - CENTRO - DISPARAR - MENU
const uint8_t TFT_LED = 1;                                                // Pin de conexión del led del TFT
const uint8_t TFT_CS_A = 6;                                               // Pin CS de la pantalla Aliados
const uint8_t TFT_CS_E = 7;                                               // Pin CS de la pantalla Enemigos

// Valores de configuracion del PWM
const uint16_t FRECUENCY_PWM = 1000; // Frecencia en Hz
const uint8_t BITS_PWM = 8;          // Número de bits (8Bits = 0 -> 255)
const uint8_t CHANNEL_PWM = 0;       // Canal de PWM

/**
 * @brief Creamos nombres para controlar cada pulsador
 *
 */
enum nombreBotones
{
  ARRIBA,
  IZQUIERDA,
  ABAJO,
  DERECHA,
  CENTRO,
  DISPARAR,
  MENU
};

/**
 * @brief Creamos nombres para el control del estado de cada casilla
 *
 */
enum nombreCasillas
{
  VACIO,       //
  AGUA,        //
  TOCADO,      //
  FRAGATA,     // Dos casillas
  DESTRUCTOR,  // Tres casillas
  SUBMARINO,   // Tres casillas
  ACORAZADO,   // Cuatro casillas
  PORTAAVIONES // Cinco casillas
};

bool estadoBoton[cantidadBotones] = {true};         // Estado de los botones
bool estadoBotonAnterior[cantidadBotones] = {true}; // Estado de los botones anterior

int8_t cursorEjeX = 0; // Posición X del cursor en el mapa
int8_t cursorEjeY = 0; // Posición Y del cursor en el mapa

uint32_t tiempoRefresco;          // Control de tiempo para refrescar las pantallas
uint32_t intervaloRefersco = 100; // Tiempo de refesco de las pantallas

const uint8_t mapaVacio[8][8] = { // Creamos un mapa con todos los valores vacíos
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0};
;
uint8_t mapaAliado[8][8] = {VACIO}; // Mapa de nuestros barcos
// uint8_t mapaEnemigo[8][8] = {VACIO};     // Mapa de los barcos del enemigo

uint8_t mapaEnemigo[8][8] = {
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0};

const uint16_t ejeLetras[8] = {27, 52, 77, 102, 127, 152, 177, 202};     // Número en píxeles del eje X de cada cuadro
const uint16_t ejeNumeros[8] = {111, 136, 161, 186, 211, 236, 261, 286}; // Número en píxeles del eje Y de cada cuadro

const char indiceLetras[8] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};  // Caracteres a mostrar de cada columna del mapa
const char indiceNumeros[8] = {'1', '2', '3', '4', '5', '6', '7', '8'}; // Caracteres a mostrar de cada número del mapa

void iniciarI2C();
void reproduce();                                           // Reproduce el sonido
void cambiarVolumen(String datos);                          // Cambia el volumen
void pantallaInicio();                                      // Mostrar imágenes en pantalla de inicio haciendo un fundido con el brillo
void imagenEstaticaInicio();                                // Crear imágen estática donde se quedará durante toda la partida
void controlBotones();                                      // Controlar el botón que ha sido pulsado e iniciar la acción correspondiente
uint16_t colorCasillaAnteriorEnemigo(uint8_t X, uint8_t Y); // Establece el color de la casilla
void displayAliado();                                       // Mostrar la imagen en el display Aliado
void displayEnemigo();                                      // Mostrar la imagen en el display Enemigo
void finPartida();

TFT_eSPI tft = TFT_eSPI(); // Creamos objeto de una pantalla (se utilizarán las dos cambiando el estado del pin CS)

void setup()
{
  Serial.begin(115200);

  iniciarI2C();

  // PWM con VSC
  ledcAttachPin(TFT_LED, CHANNEL_PWM);
  ledcSetup(CHANNEL_PWM, FRECUENCY_PWM, BITS_PWM);
  ledcWrite(CHANNEL_PWM, 0);

  for (uint8_t i = 0; i < cantidadBotones; i++)
  {
    pinMode(pinBotones[i], INPUT_PULLUP);
  }

  pinMode(TFT_CS_A, OUTPUT);
  pinMode(TFT_CS_E, OUTPUT);
  digitalWrite(TFT_CS_A, LOW);
  digitalWrite(TFT_CS_E, LOW);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  digitalWrite(TFT_CS_A, HIGH);
  digitalWrite(TFT_CS_E, HIGH);

  pantallaInicio();
  imagenEstaticaInicio();
}

void loop()
{
  reproduce();
  controlBotones();                                   // Copmprobamos si se ha pulsado algún botón
  if (millis() - tiempoRefresco >= intervaloRefersco) // Si pasa este tiempo actualizamos las pantallas
  {
    tiempoRefresco = millis();
    displayAliado();
    displayEnemigo();
  }
}

void iniciarI2C()
{
  i2sConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2sConfig.sample_rate = 44100;
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2sConfig.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sConfig.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB);
  i2sConfig.intr_alloc_flags = 0;
  i2sConfig.dma_buf_count = 8;
  i2sConfig.dma_buf_len = I2S_MCLK_MULTIPLE_256;
  i2sConfig.use_apll = false;
  i2sConfig.tx_desc_auto_clear = true;
  i2sConfig.fixed_mclk = 0;
  i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);

  i2s_pin_config_t pinConfig;
  pinConfig.bck_io_num = BCK;   // Pin de reloj de bit
  pinConfig.ws_io_num = LRC;    // Pin de selección de palabra
  pinConfig.data_out_num = DOU; // Pin de datos de salida
  pinConfig.data_in_num = DIN;  // No se utiliza la entrada de datos
  i2s_set_pin(I2S_NUM_0, &pinConfig);
}

void reproduce()
{
  static uint8_t sonidoAnterior;
  if (sonidoAnterior != sonido)
  {
    sonidoAnterior = sonido;
    dato = 0;
    i = 0;
    j = 0;
  }
  float ajusteVolumen = 1.0;
  if (sonido == SONIDO_AGUA || sonido == SONIDO_TOCADO)
  {
    dato = sonidoAgua[i + 1] << 8;
    tamanioBuffer = sonidoAguaTam;
  }

  if (sonido == SONIDO_EXPLOSION)
  {
    dato = sonidoTocado[i + 1] << 8;
    tamanioBuffer = sonidoTocadoTam;
    ledcWrite(CHANNEL_PWM, sonidoTocado[i + 1]);
  }

  if (sonido == SONIDO_PULSAR)
  {
    ajusteVolumen = 2;
    dato = sonidoPulsar[i + 1] << 8;
    tamanioBuffer = sonidoPulsarTam;
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
    if (sonido == SONIDO_EXPLOSION || sonido == SONIDO_AGUA || sonido == SONIDO_PULSAR)
    {
      sonido = SONIDO_NULO;
      ledcWrite(CHANNEL_PWM, 255);
    }

    if (sonido == SONIDO_TOCADO)
    {
      sonido = SONIDO_EXPLOSION;
    }
  }
}

void cambiarVolumen(String datos)
{
  String valor = "";
  valor += datos[1];
  valor += datos[2];
  valor += datos[3];
  uint8_t nuevoValor = valor.toInt();
  if (nuevoValor > 50)
    nuevoValor = 50;
  if (nuevoValor >= 0 && nuevoValor <= 100)
    volumen = nuevoValor / 100.0;
}

/**
 * @brief Mostramos una imagen en cada pantalla durante poco tiempo haciendo un fundido de inicio y fin
 *
 */
void pantallaInicio()
{
  digitalWrite(TFT_CS_A, LOW);                        // Habilitamos pantalla Aliados
  tft.setSwapBytes(true);                             // Habilitamos la escritura de bytes para la imagen
  tft.pushImage(0, 0, 240, 320, imagenInicioAliados); // Mostramos la imagen de bienvenida
  digitalWrite(TFT_CS_A, HIGH);                       // Deshabilitamos pantalla Aliados

  digitalWrite(TFT_CS_E, LOW);                         // Habilitamos pantalla enemigos
  tft.setSwapBytes(true);                              // Habilitamos la escritura de bytes para la imagen
  tft.pushImage(0, 0, 240, 320, imagenInicioEnemigos); // Mostramos la imagen de bienvenida
  digitalWrite(TFT_CS_E, HIGH);                        // Deshabilitamos pantalla Enemigos

  for (uint8_t i = 1; i < 254; i++) // Creamos un aumento en la iluminación de las pantallas
  {
    ledcWrite(CHANNEL_PWM, i); // Cambiamos el canal del PWM al valor del bucle
    delay(5);
  }

  delay(2000); // Hacemos una pausa

  for (uint8_t i = 255; i > 0; i--) // Creamos un fundido en la iluminación de las pantallas
  {
    ledcWrite(CHANNEL_PWM, i); // Cambiamos el canal del PWM al valor del bucle
    delay(5);
  }

  digitalWrite(TFT_CS_A, LOW);  // Habilitar pantalla Aliados
  digitalWrite(TFT_CS_E, LOW);  // Habilitar pantalla Enemigos
  tft.fillScreen(TFT_BLACK);    // Fondo Negro
  digitalWrite(TFT_CS_A, HIGH); // Deshabilitamos pantalla Aliados
  digitalWrite(TFT_CS_E, HIGH); // Deshabilitamos pantalla Enemigos
}

/**
 * @brief Creamos toda la imgen que hay que mostrar y dejar de manera etática en ambas pantallas
 *
 */
void imagenEstaticaInicio()
{
  digitalWrite(TFT_CS_A, LOW); // Habilitamos pantalla Aliados
  digitalWrite(TFT_CS_E, LOW); // Habilitamos pantalla Enemigos

  tft.fillScreen(TFT_BLACK);

  for (uint8_t i = 0; i <= 8; i++) // Creamos cuadrícula completa con los índices de números y letras
  {
    uint16_t suma = i * 25; // 25 es el ancho de la cuadrícula
    if (i != 8)
    {
      tft.setTextColor(TFT_WHITE, TFT_BLACK); // Color del texto, color de fondo
      tft.setTextSize(2);                     // Tamaño del texto
      tft.setCursor(34 + suma, 90);           // Establecemos posición
      tft.print(indiceLetras[i]);             // Pintamos las letras
      tft.setCursor(10, 116 + suma);          // Establecemos posición
      tft.print(indiceNumeros[i]);            // Pintamos los números
    }
    tft.drawLine(26, 110 + suma, 226, 110 + suma, TFT_WHITE); // Dibujamos las líneas horizontales
    tft.drawLine(26 + suma, 110, 26 + suma, 310, TFT_WHITE);  // Dibujamos las líneas verticales
  }

  digitalWrite(TFT_CS_A, HIGH); // Deshabilitamos pantalla Aliados
  digitalWrite(TFT_CS_E, HIGH); // Deshabilitamos pantalla Enemigos

  for (uint8_t i = 0; i < 255; i++) // Creamos un aumento en la iluminación de las pantallas
  {
    ledcWrite(CHANNEL_PWM, i);
    delay(5); // Pequeño retardo
  }
}

/**
 * @brief Controlamos el estado de cualquier pulsador. Esta función hay que modificarla
 *
 */
void controlBotones()
{
  static uint32_t tiempoDebounce;               // Control del tiempo que irá transcurriendo de antirrebote
  static const uint32_t debounce = 200;         // Establecemos un tiempo de antirrebote
  for (uint8_t i = 0; i < cantidadBotones; i++) // Creamos un blucle para comprobar todos los botones
  {
    estadoBoton[i] = digitalRead(pinBotones[i]); // Guardamos el estado en el array
  }

  if (estadoBoton[ARRIBA] == LOW && estadoBotonAnterior[ARRIBA] == HIGH) // Botón ARRIBA pulsado
  {
    if (sonido == SONIDO_NULO)
    {
      sonido = SONIDO_PULSAR;
    }
    estadoBotonAnterior[ARRIBA] = LOW;
    tiempoDebounce = millis();
    cursorEjeY--;
    if (cursorEjeY < 0)
      cursorEjeY = 7;
  }

  if (estadoBoton[ABAJO] == LOW && estadoBotonAnterior[ABAJO] == HIGH) // Botón ABAJO pulsado
  {
    if (sonido == SONIDO_NULO)
    {
      sonido = SONIDO_PULSAR;
    }
    estadoBotonAnterior[ABAJO] = LOW;
    tiempoDebounce = millis();
    cursorEjeY++;
    if (cursorEjeY > 7)
      cursorEjeY = 0;
  }

  if (estadoBoton[IZQUIERDA] == LOW && estadoBotonAnterior[IZQUIERDA] == HIGH) // Botón IZQUIERDA pulsado
  {
    if (sonido == SONIDO_NULO)
    {
      sonido = SONIDO_PULSAR;
    }
    estadoBotonAnterior[IZQUIERDA] = LOW;
    tiempoDebounce = millis();
    cursorEjeX--;
    if (cursorEjeX < 0)
      cursorEjeX = 7;
  }

  if (estadoBoton[DERECHA] == LOW && estadoBotonAnterior[DERECHA] == HIGH) // Botón DERECHA pulsado
  {
    if (sonido == SONIDO_NULO)
    {
      sonido = SONIDO_PULSAR;
    }
    estadoBotonAnterior[DERECHA] = LOW;
    tiempoDebounce = millis();
    cursorEjeX++;
    if (cursorEjeX > 7)
      cursorEjeX = 0;
  }

  if (estadoBoton[CENTRO] == LOW && estadoBotonAnterior[CENTRO] == HIGH) // Botón CENTRO pulsado
  {
    estadoBotonAnterior[CENTRO] = LOW;
    tiempoDebounce = millis();
  }

  if (estadoBoton[MENU] == LOW && estadoBotonAnterior[MENU] == HIGH) // Botón MENU pulsado
  {
    estadoBotonAnterior[MENU] = LOW;
    tiempoDebounce = millis();
    if (finJuego)
    {
      finJuego = false;
      for (uint8_t i = 0; i < 8; i++)
      {
        for (uint8_t j = 0; j < 8; j++)
        {
          mapaEnemigo[i][j] = mapaVacio[i][j];
        }
      }
      for (uint8_t i = 255; i > 0; i--) // Creamos un fundido en la iluminación de las pantallas
      {
        ledcWrite(CHANNEL_PWM, i); // Cambiamos el canal del PWM al valor del bucle
        delay(5);
      }
      imagenEstaticaInicio();
    }
  }

  if (estadoBoton[DISPARAR] == LOW && estadoBotonAnterior[DISPARAR] == HIGH && sonido == SONIDO_NULO) // Botón DISPARAR pulsado
  {
    estadoBotonAnterior[DISPARAR] = LOW;
    tiempoDebounce = millis();

    switch (mapaEnemigo[cursorEjeX][cursorEjeY])
    {
    case VACIO:
      mapaEnemigo[cursorEjeX][cursorEjeY] = AGUA;
      sonido = SONIDO_AGUA;
      break;
    case AGUA:
      mapaEnemigo[cursorEjeX][cursorEjeY] = AGUA;
      break;
    case TOCADO:
      mapaEnemigo[cursorEjeX][cursorEjeY] = TOCADO;
      break;
    case FRAGATA:
    case DESTRUCTOR:
    case SUBMARINO:
    case ACORAZADO:
    case PORTAAVIONES:
      mapaEnemigo[cursorEjeX][cursorEjeY] = TOCADO;
      sonido = SONIDO_TOCADO;
      break;
    }
    alcanzado = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
      for (uint8_t j = 0; j < 8; j++)
      {
        if (mapaEnemigo[i][j] == TOCADO)
          alcanzado++;
        if (alcanzado == 17)
          finJuego = true;
      }
    }
  }

  if (millis() - tiempoDebounce >= debounce) // Pasado el tiempo, si se ha pulsado algún botón, reiniciar valor
  {
    for (uint8_t i = 0; i < cantidadBotones; i++)
    {
      if (estadoBotonAnterior[i] == LOW)
        estadoBotonAnterior[i] = HIGH;
    }
  }
}

/**
 * @brief Controlamos el estado de la casilla anterior según el mapa de como esté en ese momento
 *
 * @param X Coordenada del eje X
 * @param Y Coordenada del eje Y
 * @return uint16_t Color que devuelve
 */
uint16_t colorCasillaAnteriorEnemigo(uint8_t X, uint8_t Y)
{
  uint8_t valor = mapaEnemigo[X][Y];
  switch (valor)
  {
  case VACIO:
    return TFT_BLACK;
    break;
  case AGUA:
    return TFT_BLUE;
    break;
  case TOCADO:
    return TFT_RED;
    break;
  }
  return VACIO;
}

/**
 * @brief Mostramos el display de los aliados
 *
 */
void displayAliado()
{
  digitalWrite(TFT_CS_A, LOW); // Habilitamos escritura en esta pantalla

  digitalWrite(TFT_CS_A, HIGH); // Desabilitamos escritura en esta pantalla
}

/**
 * @brief Mostramos el display de los enemigos
 *
 */
void displayEnemigo()
{
  digitalWrite(TFT_CS_E, LOW); // Habilitamos escritura en esta pantalla

  static uint32_t _tiempoIntermitenciaCursor;
  static const uint32_t _intervaloIntermitenciaCursor = 250;
  static bool _cambioColor = true;
  static uint8_t _ejeX, _ejeY;

  finPartida();

  if (_ejeX != cursorEjeX || _ejeY != cursorEjeY)
  {
    _tiempoIntermitenciaCursor = millis();
    tft.fillRect(ejeLetras[cursorEjeX], ejeNumeros[cursorEjeY], 24, 24, TFT_GREEN);
    tft.fillRect(ejeLetras[_ejeX], ejeNumeros[_ejeY], 24, 24, colorCasillaAnteriorEnemigo(_ejeX, _ejeY));
    _ejeX = cursorEjeX;
    _ejeY = cursorEjeY;
  }

  if (millis() - _tiempoIntermitenciaCursor >= _intervaloIntermitenciaCursor)
  {
    _tiempoIntermitenciaCursor = millis();
    _cambioColor = !_cambioColor;
    tft.fillRect(ejeLetras[cursorEjeX], ejeNumeros[cursorEjeY], 24, 24, _cambioColor ? TFT_GREEN : colorCasillaAnteriorEnemigo(cursorEjeX, cursorEjeY));
  }
  digitalWrite(TFT_CS_E, HIGH); // Desabilitamos escritura en esta pantalla
}

void finPartida()
{
  if (finJuego && alcanzado == 17)
  {
    tft.setTextSize(4);
    tft.setCursor(0, 0);
    tft.print("YOU WIN");
  }
  else if (finJuego && alcanzado < 17)
  {
    tft.setTextSize(4);
    tft.setCursor(0, 0);
    tft.print("YOY LOST");
  }
}
