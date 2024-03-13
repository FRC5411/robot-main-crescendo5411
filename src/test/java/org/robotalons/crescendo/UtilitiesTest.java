// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
// -------------------------------------------------------------[Utilities Test]-----------------------------------------------------------//
public final class UtilitiesTest {
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Utilizes CTRE utilities to guarantee that CTRE Error codes are properly interpreted and are thrown
   */
  @Test
  void CheckErrorCTRE() {
    assertThrows((RuntimeException.class), () -> CheckCTRECode(ErrorCode.GeneralError));
    assertThrows((RuntimeException.class), () -> CheckCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved));
    assertDoesNotThrow(() -> CheckCTRECode(ErrorCode.OK));
  }

  /**
   * Checks if a given CTRE error code is equivalent with an okay error code, if not, an exception is thrown
   * @param Code Error code to check
   */
  private static void CheckCTRECode(final ErrorCode Code) {
    if (Code != ErrorCode.OK) {
      throw new RuntimeException(String.format(("%s: %s%n"), (""), Code.toString()));
    }
  }

  /**
   * Utilizes REV utilities to guarantee that CTRE Error codes are properly interpreted and are thrown
   */
  @Test
  void CheckErrorREV() {
    assertThrows((RuntimeException.class), () -> CheckREVCode(REVLibError.kError));
    assertThrows((RuntimeException.class), () -> CheckREVCode(REVLibError.kCantFindFirmware));
    assertDoesNotThrow(() -> CheckREVCode(REVLibError.kOk));
  }

  /**
   * Checks if a given REVlib error code is equivalent with an okay error code, if not, an exception is thrown
   * @param Code Error code to check
   */
  private static void CheckREVCode(final REVLibError Code) {
    if (Code != REVLibError.kOk) {
        throw new RuntimeException(String.format(("%s: %s"), (""), Code.toString()));
    }
  }
}
