import lldb
import re
from lldbsuite.test.decorators import *
from lldbsuite.test.lldbtest import TestBase
import lldbsuite.test.lldbutil as lldbutil

class TestCase(TestBase):

    @swiftTest
    def test_task_graph(self):
        self.build()
        _, _, thread, _ = lldbutil.run_to_source_breakpoint(
            self, "break here", lldb.SBFileSpec("main.swift")
        )
        frame = thread.frames[0]
        this_frame_addr = frame.GetPCAddress().GetFileAddress()

        self.runCmd("language swift task graph")
        result = self.res.GetOutput()
        # Task 2: ... main.swift:7
        #   Task 3: ... main.swift:7
        #     Task 4: ...60 at main.swift:3
        pattern = r"Task (\d+):.*at main\.swift:(\d+)"
        matches = re.findall(pattern, result)
        task_lines = [(int(task_id), int(line)) for task_id, line in matches]
        self.assertEqual((2, 7), task_lines[0])
        self.assertEqual((3, 7), task_lines[1])
        self.assertEqual((4, 3), task_lines[2])
