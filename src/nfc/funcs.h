#pragma once

#include <stdint.h>

namespace NfcA {

  /**
   * @short Calculate the UID check byte for 4 bytes of UID data
   * 
   * @details UID CLn check byte for NFC-A. 
   *   BCC is an exclusive-OR over the first 4 bytes of the SDD_RES Response.
   * 
   * @param buf: buffer containing 4 bytes of the UID
   * @return the check byte
  */
  uint8_t calcBcc(const uint8_t *buf);

  /**
   * @short Calculate the reply to SDD_REQ frames for the tag's UID
   * 
   * @details 
   * 
   * @param uid: buffer of all UID bytes
   * @param uidSize: size of UID in bytes (must be 4, 7, or 10 bytes long)
   * @param collisionLevel: which collision level to generate reply 4 (0, 1, 2)
   * @param outputBuf: generated response output (must be 5 bytes)
   * @return 0 on success, -1 for invalid parameter
  */
  int8_t genSddResponse(const uint8_t *uid, uint8_t uidSize, uint8_t collisionLevel, uint8_t *outputBuf);

};
