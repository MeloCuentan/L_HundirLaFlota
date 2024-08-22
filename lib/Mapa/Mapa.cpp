#include <Mapa.h>

#include <TFT_eSPI.h>

extern TFT_eSPI tft;

// FUNCIONES PUBLICAS
/**
 * @brief Constructor de nuevo mapa
 *
 */
Mapa::Mapa(uint8_t pinCS)
{
  //  vaciarMapa();
  _pinCS = pinCS;
  _barcosHundidos = 0;
  pinMode(_pinCS, OUTPUT);
  digitalWrite(_pinCS, HIGH);
}

/**
 * @brief Dibujar el mapa. Imágen estática
 *
 */
void Mapa::dibujarMapa()
{
  digitalWrite(_pinCS, LOW);
  tft.fillScreen(TFT_BLACK);
  for (uint8_t i = 0; i <= _NUMERO_CUADROS; i++) // Creamos cuadrícula completa con los índices de números y letras
  {
    uint16_t suma = i * _ANCHO_CUADRO;
    tft.drawLine(4, 76 + suma, 236, 76 + suma, TFT_WHITE); // Dibujamos las líneas horizontales
    tft.drawLine(4 + suma, 76, 4 + suma, 308, TFT_WHITE);  // Dibujamos las líneas verticales
  }
  digitalWrite(_pinCS, HIGH);
}

/**
 * @brief Colocar barco en el mapa
 *
 * @param fila posición de fila
 * @param columna posición de columna
 * @param horizontal orientación del barco
 * @return true se puede poner
 * @return false error en la posición
 */
bool Mapa::colocarBarco(uint8_t tamanioBarco, uint8_t fila, uint8_t columna, bool horizontal)
{
  return true;
}

/**
 * @brief Resultado de dispara a esta posición
 *
 * @param fila posición de fila
 * @param columna posición de columna
 */
bool Mapa::disparar(uint8_t fila, uint8_t columna)
{
  uint8_t valorCasilla = _tablero[fila][columna];
  switch (valorCasilla)
  {
  case _AGUA:
    _tablero[fila][columna] = _AGUA;
    return false;
    break;
  case _TOCADO:
    _tablero[fila][columna] = _TOCADO;
    return false;
    break;
  case _VACIO:
    _tablero[fila][columna] = _AGUA;
    return false;
    break;
  case _FRAGATA:
  case _DESTRUCTOR:
  case _SUBMARINO:
  case _ACORAZADO:
  case _PORTAAVIONES:
    _tablero[fila][columna] = _TOCADO;
    return true;
    break;
  }
  return false;
}

/**
 * @brief Estado de todos los barcos
 *
 * @return true Están todos hundidos
 * @return false Quedan barcos a flote
 */
bool Mapa::todosHundidos()
{
  if (_barcosHundidos == _MAX_BARCOS)
    return true;
  else
    return false;
}

void Mapa::cursorMapa(uint8_t posX, uint8_t posY)
{
  static bool cambiaColor;
  _cursorX = posX;
  _cursorY = posY;
  digitalWrite(_pinCS, LOW);
  if (_cursorAnteriorX != _cursorX || _cursorAnteriorY != _cursorY)
  {
    tft.fillRect(_POSICION_HORIZONTAL[_cursorAnteriorX], _POSICION_VERTICAL[_cursorAnteriorY], _ANCHO_CURSOR, _ANCHO_CURSOR, colorAnterior(_cursorAnteriorX, _cursorAnteriorY));
    tft.fillRect(_POSICION_HORIZONTAL[_cursorX], _POSICION_VERTICAL[_cursorY], _ANCHO_CURSOR, _ANCHO_CURSOR, TFT_GREEN);
  }

  if (millis() - _tiempoAnterior >= _intermitencia)
  {
    _tiempoAnterior = millis();
    cambiaColor = !cambiaColor;
    tft.fillRect(_POSICION_HORIZONTAL[_cursorX], _POSICION_VERTICAL[_cursorY], _ANCHO_CURSOR, _ANCHO_CURSOR, cambiaColor ? TFT_GREEN : colorAnterior(_cursorX, _cursorY));
  }
  digitalWrite(_pinCS, HIGH);
  _cursorAnteriorX = _cursorX;
  _cursorAnteriorY = _cursorY;
};

uint16_t Mapa::colorAnterior(uint8_t posX, uint8_t posY)
{
  uint16_t devuelveColor = TFT_BLACK;
  uint8_t tipoCasilla = _tablero[posX][posY];
  switch (tipoCasilla) 
  {
    case _AGUA:
      devuelveColor = TFT_BLUE;
    break;
    case _TOCADO:
      devuelveColor = TFT_RED;
    break;
    case _VACIO:
    case _FRAGATA:
    case _DESTRUCTOR:
    case _SUBMARINO:
    case _PORTAAVIONES:
      devuelveColor = TFT_BLACK;
    break;
  }
  return devuelveColor;
}

// FUNCIONES PRIVADAS
/**
 * @brief Vaciar el mapa
 *
 */
void Mapa::vaciarMapa()
{
    memset(_tablero, _VACIO, sizeof(_tablero));
}
