"""
Test that the summary of a function pointer names the function it points to.
"""

import re

import lldb
from lldbsuite.test.lldbtest import *
import lldbsuite.test.lldbutil as lldbutil


class FunctionPointerSummaryTestCase(TestBase):
    NO_DEBUG_INFO_TESTCASE = True

    def check_summary(self, variable, function):
        target = self.target()
        value = self.frame().GetValueForVariablePath(variable)
        summary = value.GetSummary()
        self.assertIsNotNone(summary, variable)
        self.assertIn("a.out`%s(int)" % function, summary)
        # On WebAssembly the pointer holds a table index, and the summary also
        # prints the code address that the index resolves to.
        address = (
            target.FindFunctions(function)[0]
            .GetFunction()
            .GetStartAddress()
            .GetLoadAddress(target)
        )
        if value.GetValueAsUnsigned() != address:
            match = re.search(r"actual=(0x[0-9a-f]+)", summary)
            self.assertIsNotNone(match, summary)
            self.assertEqual(int(match.group(1), 16), address)

    def test(self):
        self.build()
        lldbutil.run_to_source_breakpoint(
            self, "// break here", lldb.SBFileSpec("main.cpp")
        )
        self.check_summary("global_fp", "add_one")
        self.check_summary("local_fp", "add_two")
