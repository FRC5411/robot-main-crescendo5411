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

    /** Test CTRE Error Codes throw runtime expected exceptions */
    @Test
    void CheckErrorCTRE() {
        assertThrows((RuntimeException.class), () -> CheckCTRECode(ErrorCode.GeneralError));
        assertThrows(
                (RuntimeException.class),
                () -> CheckCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved));
        assertDoesNotThrow(() -> CheckCTRECode(ErrorCode.OK));
    }

    static void CheckCTRECode(final ErrorCode errorCode) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(String.format("%s: %s%n", "", errorCode.toString()));
        }
    }

    /** Test REVLib Error Codes throw runtime expected exceptions */
    @Test
    void CheckErrorREV() {
        assertThrows((RuntimeException.class), () -> CheckREVLibCode(REVLibError.kError));
        assertThrows(
                (RuntimeException.class), () -> CheckREVLibCode(REVLibError.kCantFindFirmware));
        assertDoesNotThrow(() -> CheckREVLibCode(REVLibError.kOk));
    }

    static void CheckREVLibCode(final REVLibError error) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("%s: %s", "", error.toString()));
        }
    }
}
