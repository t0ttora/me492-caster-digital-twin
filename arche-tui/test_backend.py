import copy
import tempfile
import unittest
from pathlib import Path
import backend as b

class BackendTests(unittest.TestCase):
    def test_defaults_preserve_version_inertias(self):
        self.assertEqual(b.defaults('V1')['dynamics']['drive_roll_inertia_kgm2'], .0045)
        self.assertEqual(b.defaults('V2')['dynamics']['drive_roll_inertia_kgm2'], .001125)
    def test_invalid_values_rejected(self):
        c=b.defaults('V2');c['geometry']['drive_radius_m']=-1
        with self.assertRaises(ValueError):b.validate(c)
        c=b.defaults('V2');c['initial']['body_velocity_x_mps']=float('nan')
        with self.assertRaises(ValueError):b.validate(c)
    def test_sampling_validation(self):
        c=b.defaults('V1');c['dynamics']['solver_step_s']=.003
        with self.assertRaises(ValueError):b.validate(c)
    def test_two_kernels_and_immutable_input(self):
        for version in ['V1','V2']:
            c=b.defaults(version);c['experiment']['duration_s']=.04
            if version=='V2':c['experiment']['segment_duration_s']=.02;c['experiment']['terminal_s']=.02
            before=copy.deepcopy(c)
            with tempfile.TemporaryDirectory() as d:
                p=b.execute(c,Path(d));rows=b.read_rows(p/'results.csv')
                self.assertGreaterEqual(len(rows),3)
                self.assertAlmostEqual(rows[-1]['time_s'],.04)
                self.assertTrue((p/'summary.json').exists())
                self.assertTrue((p/'config.json').exists())
            self.assertEqual(c,before)
    def test_ascii_plot_extrema_and_flat(self):
        for values in [[0,1,0],[3,3,3]]:
            chart=b.ascii_plot(list(range(3)),values,30,8)
            self.assertEqual(len(chart),8)
            self.assertTrue(any('*' in s for s in chart))


class PresetTests(unittest.TestCase):
    def test_surface_reset_and_auto_gain_scaling(self):
        c=b.defaults('V2');base=c['dynamics']['drive_friction_coefficient']
        b.select_preset(c,'surface','high_drag_synthetic');self.assertNotEqual(c['dynamics']['drive_friction_coefficient'],base)
        b.select_preset(c,'surface','reference_synthetic');self.assertEqual(c['dynamics']['drive_friction_coefficient'],base)
        old=b.active_gains(c)['wheel_p'];c['dynamics']['drive_roll_inertia_kgm2']*=2
        self.assertAlmostEqual(b.active_gains(c)['wheel_p'],2*old)

if __name__=='__main__':unittest.main()
