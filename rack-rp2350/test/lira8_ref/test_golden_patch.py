#!/usr/bin/env python3
"""Guard golden-wrapper reproducibility without starting Pd or touching fixtures."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "gen_golden_patch", HERE / "gen_golden_patch.py")
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)

WRITE_EDGE = "#X connect 55 0 30 0;\n"
GUI_RESTORE = "#X restore 32 32 pd gui;"


class GoldenPatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (HERE / "patch" / "_LIRA-8.pd").read_text()

    def generate(self, source):
        with tempfile.TemporaryDirectory(prefix="lira8-golden-test-") as tmp:
            root = Path(tmp)
            (root / "patch").mkdir()
            (root / "patch" / "_LIRA-8.pd").write_text(source)
            with patch.object(generator, "HERE", root), \
                    contextlib.redirect_stdout(io.StringIO()):
                generator.main()
            return (root / "patch" / "_LIRA-8-golden.pd").read_bytes()

    def test_generation_is_deterministic(self):
        self.assertEqual(self.generate(self.source), self.generate(self.source))

    def test_no_active_parameter_save_edge(self):
        generated = self.generate(self.source).decode()
        self.assertNotIn(WRITE_EDGE, generated)
        # Keep the write message disconnected: no inlet edge may reach it.
        gui_link = generated.split(
            "#N canvas 191 498 1414 497 gui.link 0;\n", 1)[1].split(
            "#X restore 29 750 pd gui.link;", 1)[0]
        for line in gui_link.splitlines():
            if line.startswith("#X connect "):
                self.assertNotEqual(line.split()[4], "30", line)

    def test_read_load_path_is_intact(self):
        generated = self.generate(self.source).decode()
        for line in (
            "#X msg 208 300 read ./parameters.txt;",
            "#X obj 208 326 text define \\$0-parameters;",
            "#X obj 208 279 r \\$0-loadbang-0;",
            "#X obj 235 228 s \\$0-loadbang-0;",
            "#X obj 208 249 s \\$0-loadbang-1;",
            "#X obj 208 186 loadbang;",
            "#X connect 32 0 26 0;",
            "#X connect 26 1 28 0;",
            "#X connect 26 0 27 0;",
            "#X connect 31 0 29 0;",
            "#X connect 29 0 24 0;",
        ):
            self.assertIn(line + "\n", generated)
        # Only the unsafe edge changes inside gui.link; object numbering,
        # the text store, and every load connection remain byte-for-byte.
        start = "#N canvas 191 498 1414 497 gui.link 0;\n"
        end = "#X restore 29 750 pd gui.link;"
        original_link = self.source.split(start, 1)[1].split(end, 1)[0]
        generated_link = generated.split(start, 1)[1].split(end, 1)[0]
        self.assertEqual(original_link.replace(WRITE_EDGE, ""), generated_link)

    def test_delayed_write_anchor_must_be_unique(self):
        self.assertEqual(self.source.count(WRITE_EDGE), 1)
        for replacement in ("", WRITE_EDGE * 2):
            with self.subTest(matches=replacement.count(WRITE_EDGE)):
                with self.assertRaisesRegex(ValueError, "delayed parameter-write"):
                    self.generate(self.source.replace(WRITE_EDGE, replacement))

    def test_gui_restore_anchor_must_be_unique(self):
        self.assertEqual(self.source.count(GUI_RESTORE), 1)
        for replacement in ("", GUI_RESTORE * 2):
            with self.subTest(matches=replacement.count(GUI_RESTORE)):
                with self.assertRaisesRegex(ValueError, "gui restore"):
                    self.generate(self.source.replace(GUI_RESTORE, replacement))

    def test_additional_parameter_save_inputs_are_rejected(self):
        for edge in ("#X connect 25 0 30 0;\n",
                     "#X connect 32 0 30 0;\n"):
            with self.subTest(edge=edge.strip()):
                source = self.source.replace(WRITE_EDGE, WRITE_EDGE + edge)
                with self.assertRaisesRegex(ValueError, "parameter-write input"):
                    self.generate(source)

    def test_matching_edge_outside_gui_link_is_untouched(self):
        # Pd connection indices are canvas-local, so the same edge in a
        # sibling canvas must neither satisfy nor invalidate the save anchor.
        source = self.source + WRITE_EDGE
        generated = self.generate(source).decode()
        self.assertEqual(generated.count(WRITE_EDGE), 1)
        self.assertIn(GUI_RESTORE, generated)
        with self.assertRaisesRegex(ValueError, "delayed parameter-write"):
            self.generate(self.source.replace(WRITE_EDGE, "") + WRITE_EDGE)

    def test_gui_link_boundaries_must_be_unique(self):
        for anchor in ("#N canvas 191 498 1414 497 gui.link 0;\n",
                       "#X restore 29 750 pd gui.link;"):
            for replacement in ("", anchor * 2):
                with self.subTest(anchor=anchor, count=replacement.count(anchor)):
                    with self.assertRaisesRegex(ValueError, "gui.link"):
                        self.generate(self.source.replace(anchor, replacement))

    def test_checked_in_wrapper_matches_generation(self):
        self.assertEqual(
            (HERE / "patch" / "_LIRA-8-golden.pd").read_bytes(),
            self.generate(self.source))


if __name__ == "__main__":
    unittest.main()
