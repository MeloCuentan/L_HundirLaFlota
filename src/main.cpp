/*
    Juego de Hundir La Flota

    Este juego tendrá dos pantallas TFT de 2.4" con una resolución de 240 x 320
    En la pantalla de la izquierda estarán nuestros barcos y a la derecha los barcos del enemigo

    Como controles, tendremos un mando de 4 posiciones (ARRIBA, ABAJO, IZQUIERDA, DERECHA) que también tiene un pin CENTRAL y dos botones añadidos que
  serán los de DISPARO y MENU

    El orden del menú será el siguiente:

            |  LOCAL --------> | COLOCAR BARCOS
            |                  | COMIENZA EL JUEGO
    MENU -> |
            |  OTRO JUGADOR -> | BUSCAR DISPOSITIVO
            |                  | COLOCAR BARCOS
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
  BTN_MENU
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

uint8_t numeroMenu = MENU_IMAGENES_INICIALES;

// Pines utilizados en I2S
const uint8_t BCK = 16;
const uint8_t DOU = 17;
const uint8_t LRC = 21;
const int8_t DIN = -1;

uint8_t selectorAjuste = 0;

// Variables generales
const uint8_t CUADROS_HORIZONTAL = 8;
const uint8_t CUADROS_VERTICAL = 8;
const uint8_t ANCHO_CUADRO = 25;
const uint8_t BRILLO_MINIMO = 30;      // Valor de brillo mínimo que se puede ajustar
bool finJuego = false;                 // Si ha terminado alguien el juego
float volumen = 0.5;                   // Volumen predeterminado
float brillo = 255.0;                  // Brillo predeterminado
int8_t volumenUsuario = volumen * 100; // Valor de volumen ajustado por el usuario
uint8_t alcanzado = 0;                 // Número de impactos logrados
uint8_t brilloUsuario = 100;           // Valor de brillo ajustado por el usuario
uint8_t numeroSonido = 0;              // Número del sonido a reproducir
int16_t dato = 0;                      // Valor del array

i2s_config_t i2sConfig;
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
bool botonActivado[CANTIDAD_BOTONES] = {false};

int8_t cursorEjeX = 0; // Posición X del cursor en el mapa
int8_t cursorEjeY = 0; // Posición Y del cursor en el mapa

uint32_t tiempoRefresco;          // Control de tiempo para refrescar las pantallas
uint32_t intervaloRefersco = 100; // Tiempo de refesco de las pantallas

const uint8_t MAPA_VACIO[CUADROS_HORIZONTAL][CUADROS_VERTICAL] = { // Creamos un mapa con todos los valores vacíos
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0};

uint8_t mapaAliado[CUADROS_HORIZONTAL][CUADROS_VERTICAL] = { // Mapa de nuestros barcos
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0};
// uint8_t mapaEnemigo[CUADROS_HORIZONTAL][CUADROS_VERTICAL] = {VACIO};     // Mapa de los barcos del enemigo

uint8_t mapaEnemigo[CUADROS_HORIZONTAL][CUADROS_VERTICAL] = {
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0};

const uint16_t EJE_NUMEROS[CUADROS_HORIZONTAL] = {111, 136, 161, 186, 211, 236, 261, 286}; // Número en píxeles del eje Y de cada cuadro
const uint16_t EJE_LETRAS[CUADROS_VERTICAL] = {27, 52, 77, 102, 127, 152, 177, 202};       // Número en píxeles del eje X de cada cuadro

const char INDICE_NUMEROS[CUADROS_HORIZONTAL] = {'1', '2', '3', '4', '5', '6', '7', '8'}; // Caracteres a mostrar de cada número del mapa
const char INDICE_LETRAS[CUADROS_VERTICAL] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};    // Caracteres a mostrar de cada columna del mapa

void iniciarPines();
void iniciarPantallas();
void iniciarI2S();
void pantallaInicio();       // Mostrar imágenes en pantalla de inicio haciendo un fundido con el brillo
void imagenEstaticaInicio(); // Crear imágen estática donde se quedará durante toda la partida
void controlBrillo(bool tipoBrillo);
void crearCampoGuerra(uint8_t pantalla);
void dibujaBarcosAliados();
void reproduce();                                                      // Reproduce el sonido
void cambiarVolumen(String datos);                                     // Cambia el volumen
void controlBotones();                                                 // Controlar el botón que ha sido pulsado e iniciar la acción correspondiente
uint16_t colorCasillaAnterior(uint8_t X, uint8_t Y, uint8_t pantalla); // Establece el color de la casilla anterior
void displayAliado();                                                  // Mostrar la imagen en el display Aliado
void displayEnemigo();                                                 // Mostrar la imagen en el display Enemigo
void finPartida();
void controlMenu();
void accionBotonesJuego();
void accionBotonesAjustes();
void valoresPantallaAjustes();
void cambiarVolumen(uint8_t direccion);
void cambiarBrillo(uint8_t direccion);

TFT_eSPI tft = TFT_eSPI(); // Creamos objeto de una pantalla (se utilizarán las dos cambiando el estado del pin CS)

void setup()
{
  Serial.begin(115200);

  iniciarPines();
  iniciarI2S();
  iniciarPantallas();
}

void loop()
{
  controlMenu(); // Controlamos el estado de cada menú según el botón pulsado
  controlBotones();
  reproduce();                                        // Reproduce el sonido que se haya activado
  if (millis() - tiempoRefresco >= intervaloRefersco) // Si pasa este tiempo actualizamos las pantallas
  {
    tiempoRefresco = millis();
    displayAliado();
    displayEnemigo();
    finPartida();
  }
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
  pinMode(TFT_CS_A, OUTPUT);
  pinMode(TFT_CS_E, OUTPUT);
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
  crearCampoGuerra(AMBAS_PANTALLAS);
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

/**
 * @brief Creamos la imágen estática del campo de batalla con los índices
 *
 * @param pantalla Rercibe qué pantalla es la que vamos a dibujar
 * @param datos Recibe el valor para saber si restauramos el valor anterior del campo de batalla
 */
void crearCampoGuerra(uint8_t pantalla)
{
  switch (pantalla)
  {
  case PANTALLA_ALIADOS:
    digitalWrite(TFT_CS_A, LOW);
    break;
  case PANTALLA_ENEMIGOS:
    digitalWrite(TFT_CS_E, LOW);
    break;
  default:
    digitalWrite(TFT_CS_A, LOW);
    digitalWrite(TFT_CS_E, LOW);
    break;
  }
  /*
    if (pantalla == PANTALLA_ALIADOS)
      digitalWrite(TFT_CS_A, LOW);
    else if (pantalla == PANTALLA_ENEMIGOS)
      digitalWrite(TFT_CS_E, LOW);
    else if (pantalla == AMBAS_PANTALLAS)
    {
      digitalWrite(TFT_CS_A, LOW);
      digitalWrite(TFT_CS_E, LOW);
    }
  */

  tft.fillScreen(TFT_BLACK);

  for (uint8_t i = 0; i <= CUADROS_HORIZONTAL; i++) // Creamos cuadrícula completa con los índices de números y letras
  {
    uint16_t suma = i * ANCHO_CUADRO; // 25 es el ancho de la cuadrícula

    if (i != CUADROS_VERTICAL)
    {
      tft.setTextColor(TFT_WHITE, TFT_BLACK); // Color del texto, color de fondo
      tft.setTextSize(2);                     // Tamaño del texto
      tft.setCursor(34 + suma, 90);           // Establecemos posición
      tft.print(INDICE_LETRAS[i]);            // Pintamos las letras
      tft.setCursor(10, 116 + suma);          // Establecemos posición
      tft.print(INDICE_NUMEROS[i]);           // Pintamos los números
    }

    tft.drawLine(26, 110 + suma, 226, 110 + suma, TFT_WHITE); // Dibujamos las líneas horizontales
    tft.drawLine(26 + suma, 110, 26 + suma, 310, TFT_WHITE);  // Dibujamos las líneas verticales
  }
  digitalWrite(TFT_CS_A, HIGH);
  digitalWrite(TFT_CS_E, HIGH);
}

void dibujaBarcosAliados()
{
  digitalWrite(TFT_CS_A, LOW);
  for (uint8_t i = 0; i < CUADROS_HORIZONTAL; i++)
  {
    for (uint8_t j = 0; j < CUADROS_VERTICAL; j++)
    {
      uint16_t colorTemporal = 0;
      uint8_t valorTemporal = mapaAliado[i][j];
      if (valorTemporal == VACIO)
      {
        colorTemporal = TFT_BLACK;
      }
      else if (valorTemporal == AGUA)
      {
        colorTemporal = TFT_BLUE;
      }
      else if (valorTemporal == TOCADO)
      {
        colorTemporal = TFT_RED;
      }
      else
      {
        colorTemporal = TFT_YELLOW;
      }
      tft.fillRect(EJE_LETRAS[i], EJE_NUMEROS[j], 24, 24, colorTemporal);
    }
  }
  digitalWrite(TFT_CS_A, HIGH);
}

void reproduce() // Reproduce el sonido seleccinoado
{
  static uint8_t sonidoAnterior;
  static uint32_t tamanioBuffer = 0;
  static int32_t i, j;
  float ajusteVolumen = 1.0;

  if (sonidoAnterior != numeroSonido) // Reiniciamos el contador
  {
    sonidoAnterior = numeroSonido;
    dato = 0;
    i = 0;
    j = 0;
  }

  switch (numeroSonido) // Seleccionamos el tipo de sonido
  {
    //  case SONIDO_NULO:
    //    break;
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

  /*
    if (numeroSonido == SONIDO_AGUA || numeroSonido == SONIDO_TOCADO) // Sonido del disparo
    {
      dato = sonidoAgua[i + 1] << 8 | sonidoAgua[i];
      tamanioBuffer = sonidoAguaTam;
    }

    if (numeroSonido == SONIDO_EXPLOSION) // Sonido de explosión con efecto de brillo
    {
      dato = sonidoTocado[i + 1] << 8 | sonidoTocado[i];
      tamanioBuffer = sonidoTocadoTam;
      uint8_t valorTemporal = map(sonidoTocado[i + 1], 0, 255, 0, uint8_t(brillo));
      ledcWrite(CHANNEL_PWM, valorTemporal);
    }

    if (numeroSonido == SONIDO_PULSAR) // Cuando se pulsa algún botón
    {
      ajusteVolumen = 2;
      dato = sonidoPulsar[i + 1] << 8 | sonidoPulsar[i];
      tamanioBuffer = sonidoPulsarTam;
    }
  */

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
    switch (numeroSonido)
    {
    case SONIDO_EXPLOSION:
      numeroSonido = SONIDO_NULO;
      ledcWrite(CHANNEL_PWM, uint8_t(brillo));
      break;
    case SONIDO_AGUA:
      numeroSonido = SONIDO_NULO;
      break;
    case SONIDO_PULSAR:
      numeroSonido = SONIDO_NULO;
      break;
    case SONIDO_TOCADO:
      numeroSonido = SONIDO_EXPLOSION;
      break;
    }

    /*
    if (numeroSonido == SONIDO_EXPLOSION)
    {
      numeroSonido = SONIDO_NULO;
      ledcWrite(CHANNEL_PWM, uint8_t(brillo));
    }
    else if (numeroSonido == SONIDO_AGUA || numeroSonido == SONIDO_PULSAR)
    {
      numeroSonido = SONIDO_NULO;
    }

    if (numeroSonido == SONIDO_TOCADO)
    {
      numeroSonido = SONIDO_EXPLOSION;
    }
    */
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
      botonActivado[i] = true;
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

/**
 * @brief Controlamos el estado de la casilla anterior según el mapa de como esté en ese momento
 *
 * @param X Coordenada del eje X
 * @param Y Coordenada del eje Y
 * @param pantalla Enviamos a qué pantalla es la que queremos obtener el dato
 * @return uint16_t Color que devuelve
 */
uint16_t colorCasillaAnterior(uint8_t X, uint8_t Y, uint8_t pantalla)
{
  uint8_t valor = 0;
  if (pantalla == PANTALLA_ENEMIGOS)
    valor = mapaEnemigo[X][Y];
  else if (pantalla == PANTALLA_ALIADOS)
    valor = mapaAliado[X][Y];

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

void displayAliado() // Mostramos el display de los aliados
{
  digitalWrite(TFT_CS_A, LOW); // Habilitamos escritura en esta pantalla

  digitalWrite(TFT_CS_A, HIGH); // Desabilitamos escritura en esta pantalla
}

void displayEnemigo() // Mostramos el display de los enemigos
{
  digitalWrite(TFT_CS_E, LOW); // Habilitamos escritura en esta pantalla

  static uint32_t _tiempoIntermitenciaCursor;
  static const uint32_t _intervaloIntermitenciaCursor = 250;
  static bool _cambioColor = true;
  static uint8_t _ejeX, _ejeY;

  if (_ejeX != cursorEjeX || _ejeY != cursorEjeY)
  {
    _tiempoIntermitenciaCursor = millis();

    tft.fillRect(EJE_LETRAS[cursorEjeX], EJE_NUMEROS[cursorEjeY], 24, 24, TFT_GREEN);
    tft.fillRect(EJE_LETRAS[_ejeX], EJE_NUMEROS[_ejeY], 24, 24, colorCasillaAnterior(_ejeX, _ejeY, PANTALLA_ENEMIGOS));
    _ejeX = cursorEjeX;
    _ejeY = cursorEjeY;
  }

  if (numeroMenu == MENU_JUEGO)
  {
    if (millis() - _tiempoIntermitenciaCursor >= _intervaloIntermitenciaCursor)
    {
      _tiempoIntermitenciaCursor = millis();
      _cambioColor = !_cambioColor;
      tft.fillRect(EJE_LETRAS[cursorEjeX], EJE_NUMEROS[cursorEjeY], 24, 24, _cambioColor ? TFT_GREEN : colorCasillaAnterior(cursorEjeX, cursorEjeY, PANTALLA_ENEMIGOS));
    }
  }
  else
  {
    tft.fillRect(EJE_LETRAS[cursorEjeX], EJE_NUMEROS[cursorEjeY], 24, 24, colorCasillaAnterior(cursorEjeX, cursorEjeY, PANTALLA_ENEMIGOS));
  }

  digitalWrite(TFT_CS_E, HIGH); // Desabilitamos escritura en esta pantalla
}

void finPartida()
{
  if (finJuego && alcanzado == 17 && numeroMenu != MENU_AJUSTES)
  {
    digitalWrite(TFT_CS_E, LOW);
    digitalWrite(TFT_CS_A, LOW);

    tft.setTextSize(4);
    tft.setCursor(36, 29);
    tft.print(F("YOU WIN"));

    digitalWrite(TFT_CS_E, HIGH);
    digitalWrite(TFT_CS_A, HIGH);
  }
  else if (finJuego && alcanzado < 17)
  {
    digitalWrite(TFT_CS_E, LOW);
    digitalWrite(TFT_CS_A, LOW);

    tft.setTextSize(4);
    tft.setCursor(24, 29);
    tft.print(F("YOU LOST"));

    digitalWrite(TFT_CS_E, HIGH);
    digitalWrite(TFT_CS_A, HIGH);
  }
}

void controlMenu()
{
  switch (numeroMenu)
  {
  case MENU_IMAGENES_INICIALES:
    pantallaInicio();
    numeroMenu = MENU_INICIO;
    break;
  case MENU_INICIO:
    imagenEstaticaInicio();
    numeroMenu = MENU_JUEGO;
    break;
  case MENU_JUEGO:
    accionBotonesJuego();
    break;
  case MENU_AJUSTES:
    valoresPantallaAjustes();
    accionBotonesAjustes();
    break;
  }
}

void mostrarDisplayAjustes()
{
  digitalWrite(TFT_CS_A, LOW); // Habilitamos escritura en esta pantalla
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(24, 0);
  tft.print(F("NUEVO JUEGO"));
  tft.setCursor(24, 16);
  tft.print(F("VOLUMEN:"));
  tft.setCursor(24, 32);
  tft.print(F("BRILLO:"));
  digitalWrite(TFT_CS_A, HIGH); // Desabilitamos escritura en esta pantalla
}

void accionBotonesJuego()
{
  if (botonActivado[BTN_ARRIBA]) // Movemos el cursor hacia arriba
  {
    botonActivado[BTN_ARRIBA] = false;
    if (numeroSonido == SONIDO_NULO)
    {
      numeroSonido = SONIDO_PULSAR;
    }
    cursorEjeY--;
    if (cursorEjeY < 0)
      cursorEjeY = 7;
  }
  if (botonActivado[BTN_ABAJO]) // Movemos el cursor hacia abajo
  {
    botonActivado[BTN_ABAJO] = false;
    if (numeroSonido == SONIDO_NULO)
    {
      numeroSonido = SONIDO_PULSAR;
    }
    cursorEjeY++;
    if (cursorEjeY > 7)
      cursorEjeY = 0;
  }
  if (botonActivado[BTN_IZQUIERDA]) // Movemos el cursor a la izquierda
  {
    botonActivado[BTN_IZQUIERDA] = false;
    if (numeroSonido == SONIDO_NULO)
    {
      numeroSonido = SONIDO_PULSAR;
    }
    cursorEjeX--;
    if (cursorEjeX < 0)
      cursorEjeX = 7;
  }
  if (botonActivado[BTN_DERECHA]) // Movemos el cursor a la derecha
  {
    botonActivado[BTN_DERECHA] = false;
    if (numeroSonido == SONIDO_NULO)
    {
      numeroSonido = SONIDO_PULSAR;
    }
    cursorEjeX++;
    if (cursorEjeX > 7)
      cursorEjeX = 0;
  }
  if (botonActivado[BTN_CENTRO]) // Apagamos las pantallas
  {
    botonActivado[BTN_CENTRO] = false;
  }
  if (botonActivado[BTN_DISPARAR]) // Disparamos en la posición del cursor
  {
    botonActivado[BTN_DISPARAR] = false;
    if (numeroSonido == SONIDO_NULO)
    {
      switch (mapaEnemigo[cursorEjeX][cursorEjeY]) // Ponemos el color de la casilla según corresponda
      {
      case VACIO:
        mapaEnemigo[cursorEjeX][cursorEjeY] = AGUA;
        numeroSonido = SONIDO_AGUA;
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
        numeroSonido = SONIDO_TOCADO;
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
  }
  if (botonActivado[BTN_MENU]) // Iremos al menú principal para cambiar ajustes o terminar partida
  {
    botonActivado[BTN_MENU] = false;
    numeroMenu = MENU_AJUSTES;
    selectorAjuste = AJUSTE_NUEVO_JUEGO;
    mostrarDisplayAjustes();
    valoresPantallaAjustes();
    delay(500);
  }
}

void accionBotonesAjustes()
{
  uint8_t direccion = 0;

  if (botonActivado[BTN_ARRIBA]) // Movemos el cursor hacia arriba
  {
    botonActivado[BTN_ARRIBA] = false;
    if (selectorAjuste > 0)
      selectorAjuste--;
    numeroSonido = SONIDO_PULSAR;
  }

  if (botonActivado[BTN_ABAJO]) // Movemos el cursor hacia abajo
  {
    botonActivado[BTN_ABAJO] = false;
    if (selectorAjuste < LIMITE_AJUSTE - 1)
      selectorAjuste++;
    numeroSonido = SONIDO_PULSAR;
  }

  if (botonActivado[BTN_IZQUIERDA]) // Movemos el cursor a la izquierda
  {
    botonActivado[BTN_IZQUIERDA] = false;
    direccion = 1;
    numeroSonido = SONIDO_PULSAR;
  }

  if (botonActivado[BTN_DERECHA]) // Movemos el cursor a la derecha
  {
    botonActivado[BTN_DERECHA] = false;
    direccion = 2;
    numeroSonido = SONIDO_PULSAR;
  }

  if (botonActivado[BTN_CENTRO]) // Apagamos las pantallas
  {
    botonActivado[BTN_CENTRO] = false;
  }

  if (botonActivado[BTN_DISPARAR]) // Disparamos en la posición del cursor
  {
    botonActivado[BTN_DISPARAR] = false;
    if (selectorAjuste == AJUSTE_NUEVO_JUEGO)
    {
      numeroMenu = MENU_INICIO;
      finJuego = false;
      for (uint8_t i = 0; i < 8; i++)
      {
        for (uint8_t j = 0; j < 8; j++)
        {
          mapaEnemigo[i][j] = MAPA_VACIO[i][j];
        }
      }
      controlBrillo(BAJA_BRILLO);
      dibujaBarcosAliados();
    }
  }

  if (botonActivado[BTN_MENU]) // Iremos al menú principal para cambiar ajustes o terminar partida
  {
    botonActivado[BTN_MENU] = false;
    numeroMenu = MENU_JUEGO;
    crearCampoGuerra(PANTALLA_ALIADOS);
    dibujaBarcosAliados();
    delay(500);
  }

  switch (selectorAjuste)
  {
  case AJUSTE_VOLUMEN:
    cambiarVolumen(direccion);
    break;
  case AJUSTE_BRILLO:
    cambiarBrillo(direccion);
    break;
  }
}

void cambiarVolumen(uint8_t direccion)
{
  switch (direccion)
  {
  case 1:
    if (volumenUsuario > 0)
    {
      volumenUsuario -= 10;
      valoresPantallaAjustes();
    }
    break;
  case 2:
    if (volumenUsuario < 100)
    {
      volumenUsuario += 10;
      valoresPantallaAjustes();
    }
    break;
  }
}

void cambiarBrillo(uint8_t direccion)
{
  switch (direccion)
  {
  case 1:
    if (brilloUsuario > BRILLO_MINIMO)
    {
      brilloUsuario -= 10;
      valoresPantallaAjustes();
    }
    break;
  case 2:
    if (brilloUsuario < 100)
    {
      brilloUsuario += 10;
      valoresPantallaAjustes();
    }
    break;
  }
}

void valoresPantallaAjustes()
{
  static uint32_t tiempoIntermitencia, intervaloIntermitencia = 200;
  static bool intermitencia;

  if (millis() - tiempoIntermitencia >= intervaloIntermitencia)
  {
    tiempoIntermitencia = millis();
    intermitencia = !intermitencia;

    digitalWrite(TFT_CS_A, LOW);

    tft.setTextSize(2);

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
    brillo = float(brilloUsuario / 100.0) * 255.0;
    ledcWrite(CHANNEL_PWM, uint8_t(brillo));

    digitalWrite(TFT_CS_A, HIGH);
  }
}
