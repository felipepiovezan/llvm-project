"""
Tests the MultiBreakpoint packet, this test runs against whichever debug server
the platform provides (debugserver on macOS, lldb-server elsewhere).
"""

import lldb
from lldbsuite.test.decorators import *
from lldbsuite.test.lldbtest import *
from lldbsuite.test import lldbutil


@skipUnlessDarwin # Remove once lldbsever support is implemented.
@skipIfOutOfTreeDebugserver
class TestMultiBreakpoint(TestBase):
    def send_packet(self, packet_str):
        self.runCmd(f"process plugin packet send {packet_str}", check=False)
        output = self.res.GetOutput()
        reply = output.split("\n")
        # The output is of the form:
        #  packet: <packet_str>
        #  response: <response>
        packet_line = None
        response_line = None
        for line in reply:
            line = line.strip()
            if line.startswith("packet:"):
                packet_line = line
            elif line.startswith("response:"):
                response_line = line
        self.assertIsNotNone(packet_line, f"No 'packet:' line in output: {output}")
        self.assertIsNotNone(response_line, f"No 'response:' line in output: {output}")
        return response_line[len("response:") :].strip()

    def check_invalid_packet(self, packet_str):
        """Assert that sending a malformed packet returns an error."""
        reply = self.send_packet(packet_str)
        self.assertMultiResponse(reply, ["error"])

    def assertMultiResponse(self, reply, expected):
        """Assert a ';'-separated multi-response matches the expected pattern.

        Each element of `expected` is either "OK" for an exact match, or
        "error" to accept any error response (starting with 'E')."""
        parts = reply.split(";")
        self.assertEqual(len(parts), len(expected),
                         f"Expected {len(expected)} responses, got {len(parts)}: {reply}")
        for i, (actual, exp) in enumerate(zip(parts, expected)):
            if exp == "OK":
                self.assertEqual(actual, "OK", f"Response {i}: expected OK, got {actual}")
            elif exp == "error":
                self.assertTrue(actual.startswith("E"),
                                f"Response {i}: expected error, got {actual}")
            else:
                self.fail(f"Bad expected value '{exp}' at index {i}")

    def get_function_address(self, name):
        """Return the hex address of a function as a string (no 0x prefix)."""
        funcs = self.target.FindFunctions(name)
        self.assertGreater(len(funcs), 0, f"Could not find function '{name}'")
        addr = funcs[0].GetSymbol().GetStartAddress().GetLoadAddress(self.target)
        self.assertNotEqual(addr, lldb.LLDB_INVALID_ADDRESS)
        return f"{addr:x}"

    def test_multi_breakpoint(self):
        self.build()
        source_file = lldb.SBFileSpec("main.c")
        self.target, process, thread, bkpt = lldbutil.run_to_source_breakpoint(
            self, "break here", source_file
        )

        # Verify the server advertises MultiBreakpoint support.
        reply = self.send_packet("qSupported")
        self.assertIn("MultiBreakpoint+", reply)

        addr_a = self.get_function_address("func_a")
        addr_b = self.get_function_address("func_b")
        addr_c = self.get_function_address("func_c")

        # For breakpoint kind, use 4 on AArch64 (4-byte instruction), 1 elsewhere.
        arch = self.getArchitecture()
        if arch in ["arm64", "aarch64"]:
            bp_kind = "4"
        else:
            bp_kind = "1"

        # --- Malformed packets ---

        # No colon.
        self.check_invalid_packet("MultiBreakpoint")
        # Empty body after colon.
        self.check_invalid_packet("MultiBreakpoint:")
        # Missing type digit.
        self.check_invalid_packet(f"MultiBreakpoint:Z")
        # Missing address.
        self.check_invalid_packet(f"MultiBreakpoint:Z0,")
        # Missing kind.
        self.check_invalid_packet(f"MultiBreakpoint:Z0,{addr_a}")
        # Missing kind (has trailing comma but no value).
        self.check_invalid_packet(f"MultiBreakpoint:Z0,{addr_a},")
        # Invalid stoppoint type.
        self.check_invalid_packet(f"MultiBreakpoint:Z9,{addr_a},{bp_kind}")
        # Completely garbled.
        self.check_invalid_packet("MultiBreakpoint:hello")
        # Just a semicolon.
        self.check_invalid_packet("MultiBreakpoint:;")

        # --- Set a single breakpoint ---
        reply = self.send_packet(f"MultiBreakpoint:Z0,{addr_a},{bp_kind}")
        self.assertEqual(reply, "OK")

        # --- Remove the breakpoint we just set ---
        reply = self.send_packet(f"MultiBreakpoint:z0,{addr_a},{bp_kind}")
        self.assertEqual(reply, "OK")

        # --- Set multiple breakpoints at once ---
        reply = self.send_packet(
            f"MultiBreakpoint:Z0,{addr_a},{bp_kind};Z0,{addr_b},{bp_kind};Z0,{addr_c},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK;OK")

        # --- Remove multiple breakpoints at once ---
        reply = self.send_packet(
            f"MultiBreakpoint:z0,{addr_a},{bp_kind};z0,{addr_b},{bp_kind};z0,{addr_c},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK;OK")

        # --- Mixed set and remove in one packet ---
        # Set two breakpoints first.
        reply = self.send_packet(
            f"MultiBreakpoint:Z0,{addr_a},{bp_kind};Z0,{addr_b},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK")
        # Remove one, set another, remove the other.
        reply = self.send_packet(
            f"MultiBreakpoint:z0,{addr_a},{bp_kind};Z0,{addr_c},{bp_kind};z0,{addr_b},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK;OK")
        # Clean up.
        reply = self.send_packet(f"MultiBreakpoint:z0,{addr_c},{bp_kind}")
        self.assertEqual(reply, "OK")

        # --- Set the same breakpoint twice
        reply = self.send_packet(
            f"MultiBreakpoint:Z0,{addr_a},{bp_kind};Z0,{addr_a},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK")
        # Clean up both.
        reply = self.send_packet(
            f"MultiBreakpoint:z0,{addr_a},{bp_kind};z0,{addr_a},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK")

        # --- Set the same breakpoint twice, but remove it thrice.
        reply = self.send_packet(
            f"MultiBreakpoint:Z0,{addr_a},{bp_kind};Z0,{addr_a},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK")
        reply = self.send_packet(
            f"MultiBreakpoint:z0,{addr_a},{bp_kind};z0,{addr_a},{bp_kind};z0,{addr_a},{bp_kind}"
        )
        self.assertMultiResponse(reply, ["OK", "OK", "error"])

        # --- Set and remove the same address in a single packet ---
        # The spec requires requests to be executed in order, so the set
        # should succeed and the subsequent remove should find and clear it.
        reply = self.send_packet(
            f"MultiBreakpoint:Z0,{addr_a},{bp_kind};z0,{addr_a},{bp_kind}"
        )
        self.assertEqual(reply, "OK;OK")

        # --- Remove a breakpoint that was never set ---
        reply = self.send_packet(f"MultiBreakpoint:z0,{addr_b},{bp_kind}")
        self.assertMultiResponse(reply, ["error"])
