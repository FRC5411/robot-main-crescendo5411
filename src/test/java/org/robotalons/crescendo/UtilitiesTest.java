// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//




// -------------------------------------------------------------[Utilities Test]-----------------------------------------------------------//
public final class UtilitiesTest {

    // /** Test CTRE Error Codes throw runtime expected exceptions */
    // @Test
    // void CheckErrorCTRE() {
    //     assertThrows((RuntimeException.class), () -> CheckCTRECode(ErrorCode.GeneralError));
    //     assertThrows(
    //             (RuntimeException.class),
    //             () -> CheckCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved));
    //     assertDoesNotThrow(() -> CheckCTRECode(ErrorCode.OK));
    // }

    // static void CheckCTRECode(final ErrorCode ErrorCode) {
    //     if (ErrorCode != ErrorCode.OK) {
    //         throw new RuntimeException(String.format("%s: %s%n", "", ErrorCode.toString()));
    //     }
    // }

    // /** Test REVLib Error Codes throw runtime expected exceptions */
    // @Test
    // void CheckErrorREV() {
    //     assertThrows((RuntimeException.class), () -> CheckREVLibCode(REVLibError.kError));
    //     assertThrows(
    //             (RuntimeException.class), () -> CheckREVLibCode(REVLibError.kCantFindFirmware));
    //     assertDoesNotThrow(() -> CheckREVLibCode(REVLibError.kOk));
    // }

    // static void CheckREVLibCode(final REVLibError error) {
    //     if (error != REVLibError.kOk) {
    //         throw new RuntimeException(String.format("%s: %s", "", error.toString()));
    //     }
    // }
}
