/***********************************************************************
Copyright (C) 2022 Joseph Charamut

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

void logInit();
void logStr(const char *str);
void logChr(char c);
void logInt(uint8_t v);
void logHex(uint8_t v);
void hexdump(uint8_t *buf, uint8_t sz);

#ifdef __cplusplus
}
#endif
