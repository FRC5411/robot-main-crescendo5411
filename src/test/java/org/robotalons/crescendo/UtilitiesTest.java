// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
// -------------------------------------------------------------[Utilities Test]-----------------------------------------------------------//
/**
 * Test for simple exceptions to be thrown when an error arises at runtime with CTRE and REV error codes.
 * 
 * @author Cody Washington
 */
public final class UtilitiesTest {
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Utilizes CTRE utilities to guarantee that CTRE Error codes are properly interpreted and are thrown
   */
  @Test
  void checkErrorCTRE() {
    assertThrows((RuntimeException.class), () -> checkCTRECode(ErrorCode.GeneralError));
    assertThrows((RuntimeException.class), () -> checkCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved));
    assertDoesNotThrow(() -> checkCTRECode(ErrorCode.OK));
  }

  /**
   * Checks if a given CTRE error code is equivalent with an okay error code, if not, an exception is thrown
   * @param Code Error code to check
   */
  private static void checkCTRECode(final ErrorCode Code) {
    if (Code != ErrorCode.OK) {
      throw new RuntimeException(String.format(("%s: %s%n"), (""), Code.toString()));
    }
  }

  /**
   * Utilizes REV utilities to guarantee that CTRE Error codes are properly interpreted and are thrown
   */
  @Test
  void checkErrorREV() {
    assertThrows((RuntimeException.class), () -> checkREVCode(REVLibError.kError));
    assertThrows((RuntimeException.class), () -> checkREVCode(REVLibError.kCantFindFirmware));
    assertDoesNotThrow(() -> checkREVCode(REVLibError.kOk));
  }

  /**
   * Checks if a given REVlib error code is equivalent with an okay error code, if not, an exception is thrown
   * @param Code Error code to check
   */
  private static void checkREVCode(final REVLibError Code) {
    if (Code != REVLibError.kOk) {
        throw new RuntimeException(String.format(("%s: %s"), (""), Code.toString()));
    }
  }
}
