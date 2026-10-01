import unittest
from unittest.mock import patch
from mrbobus_console.radio import allow_source


class RadioSourceTests(unittest.TestCase):
    def test_strict_sources(self):
        state=dict(mode='manual', connected=True, gear=1, held=False)
        with patch('mrbobus_console.radio.read_radio', return_value=state):
            self.assertTrue(allow_source(True));self.assertFalse(allow_source(False))
            state['mode']='auto'
            self.assertFalse(allow_source(True));self.assertTrue(allow_source(False))

    def test_stop_blocks_both_sources(self):
        with patch('mrbobus_console.radio.read_radio', return_value=dict(held=True)):
            self.assertFalse(allow_source(True));self.assertFalse(allow_source(False))

    def test_dead_service_blocks_both_sources(self):
        with patch('mrbobus_console.radio.read_radio', return_value=None):
            self.assertFalse(allow_source(True));self.assertFalse(allow_source(False))

    def test_no_radio_allows_auto_only(self):
        with patch('mrbobus_console.radio.read_radio', return_value=dict(mode='auto',connected=False,gear=0,held=False)):
            self.assertFalse(allow_source(True));self.assertTrue(allow_source(False))
