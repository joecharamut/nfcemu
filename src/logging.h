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
