#ifndef MAPA_H
#define MAPA_H

#include <Arduino.h>


class Mapa
{

public:
  Mapa(uint8_t pinCS);
  void dibujarMapa();
  bool colocarBarco(uint8_t tamanioBarco, uint8_t fila, uint8_t columna, bool horizontal);
  bool disparar(uint8_t fila, uint8_t columna);
  bool todosHundidos();
  void cursorMapa(uint8_t posX, uint8_t posY);
  void vaciarMapa();
  uint16_t colorAnterior(uint8_t posX, uint8_t posY);

private:
  static const uint8_t _FRAGATA = 3; // Tamaño de dos casillas
  static const uint8_t _DESTRUCTOR = 4;  // Tamaño de tres casillas
  static const uint8_t _SUBMARINO = 5;   // Tamaño de tres casillas
  static const uint8_t _ACORAZADO = 6;   // Tamaño de cuatro casillas
  static const uint8_t _PORTAAVIONES = 7; // Tamaño de cinco casillas
  static const uint8_t _MAX_FILAS = 8;
  static const uint8_t _MAX_COLUMNAS = 8;
  static const uint8_t _MAX_BARCOS = 5;
  static const uint8_t _MAX_IMPACTOS = 17;
  static const uint8_t _VACIO = 0;
  static const uint8_t _AGUA = 1;
  static const uint8_t _TOCADO = 2;
  static const uint8_t _NUMERO_CUADROS = 8; // Número de cuadros del mapa en horizontal y vertical
  static const uint8_t _ANCHO_CURSOR = 28;  // Ancho del cuadrado del cursor
  static const uint8_t _ANCHO_CUADRO = 29;  // Ancho de cada cuadro

  const uint16_t _POSICION_HORIZONTAL[_MAX_COLUMNAS] = {5, 34, 63, 92, 121, 150, 179, 208}; // Posición horizontal en píxeles del cursor
  const uint16_t _POSICION_VERTICAL[_MAX_FILAS] = {77, 106, 135, 164, 193, 222, 251, 280};  // Posición vertical en píxeles del cursor

  uint32_t _tiempoAnterior;
  uint32_t _intermitencia = 200;
  uint8_t _cursorX, _cursorY;
  uint8_t _cursorAnteriorX, _cursorAnteriorY;
  uint8_t _pinCS;
  uint8_t _barcosHundidos;
  uint8_t _impactos = 0;
  uint8_t _tablero[_MAX_FILAS][_MAX_COLUMNAS]  = {
    0, 0, 0, 0, 6, 6, 6, 6,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 5, 0, 7, 7, 7, 7, 7,
    0, 5, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 3, 3, 0, 0, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0}; 

};

#endif