import json,tempfile,unittest
from pathlib import Path
import backend as b
import dashboard as d

class DashboardTests(unittest.TestCase):
    def test_menu_exposes_run_and_parameters(self):
        actions=[item[0] for item in d.MENU]
        for action in ['v1','v2','run','experiment','geometry','physics','initial','controller','archive','data','plot','html','quit']:self.assertIn(action,actions)
    def test_braille_constant_and_shared_scale(self):
        grids,bounds=d.raster([([0,1],[0,0]),([0,1],[10,10])],20,5)
        self.assertEqual(len(grids),2)
        self.assertEqual(len(grids[0]),5)
        self.assertTrue(any(c!=' ' for row in grids[0] for c in row))
        self.assertLess(bounds[2],0);self.assertGreater(bounds[3],10)
        self.assertNotEqual(grids[0],grids[1])
    def test_equal_aspect_uses_square_braille_pixels(self):
        _,bounds=d.raster([([0,1],[0,0])],40,10,equal=True)
        self.assertAlmostEqual((bounds[1]-bounds[0])/79,(bounds[3]-bounds[2])/39)
    def test_vectors_become_individual_editable_coordinates(self):
        fields=d.parameter_fields(b.defaults('V2'),'geometry')
        parts=[x for x in fields if x[0]=='caster_pivot_m']
        self.assertEqual([x[1] for x in parts],[0,1,2])
        self.assertTrue(all('JSON' not in x[2] for x in fields))
    def test_live_observation_preserves_both_kernel_results(self):
        for version in ['V1','V2']:
            c=b.defaults(version)
            if version=='V1':c['experiment']['duration_s']=.04
            else:c['experiment'].update(segment_duration_s=.02,terminal_s=.02)
            with tempfile.TemporaryDirectory() as tmp:
                a=b.execute(c,Path(tmp)/'a');z=b.execute(c,Path(tmp)/'z',live=True)
                self.assertEqual(b.read_rows(a/'results.csv'),b.read_rows(z/'results.csv'))
                live=json.loads((z/'live.json').read_text());self.assertAlmostEqual(live['progress'],1)
                self.assertEqual(live['rows'][-1]['time_s'],.04)

if __name__=='__main__':unittest.main()

class LayoutTests(unittest.TestCase):
    def test_startup_has_one_click_examples_and_custom_run(self):
        from qa_dashboard import Screen
        for w,h in [(80,24),(125,40)]:
            screen=Screen(w,h);ui=d.Dashboard(screen);ui.draw()
            self.assertEqual(ui.panel,'experiment')
            for target in ['v1','v2','experiment_all','run','help']:
                self.assertTrue(any(hit[-1]==target for hit in ui.hits),target)
            self.assertIn('Run with these inputs',screen.contents())
    def test_example_resets_only_selected_model_before_start(self):
        from qa_dashboard import Screen
        from unittest.mock import patch
        ui=d.Dashboard(Screen(),autoload=False)
        ui.configs['V1']['geometry']['caster_trail_m']=.09
        ui.c['geometry']['caster_trail_m']=.08
        with patch.object(ui,'start') as start:
            ui.action(('example','V2'));start.assert_called_once()
        self.assertEqual(ui.c,b.defaults('V2'))
        self.assertEqual(ui.configs['V1']['geometry']['caster_trail_m'],.09)
    def test_entire_guide_is_available_without_clipping(self):
        from qa_dashboard import Screen
        for w,h in [(80,24),(125,40)]:
            ui=d.Dashboard(Screen(w,h),autoload=False);ui.panel='help'
            width=w-max(24,min(30,w//4))-5;capacity=max(1,h-18)
            pages=d.guide_pages(width,capacity)
            for i,(title,lines) in enumerate(pages):
                ui.help_page=i;ui.draw()
                self.assertLessEqual(len(lines),capacity)
                for line in lines:self.assertIn(line,ui.s.contents())
                self.assertTrue(any(hit[-1]==('help_page',1) for hit in ui.hits))
            self.assertEqual({title for title,_ in pages},{title for title,_ in d.GUIDE})
    def test_all_experiment_inputs_remain_available(self):
        from qa_dashboard import Screen
        ui=d.Dashboard(Screen(),autoload=False)
        for version in ['V1','V2']:
            ui.version=version;ui.panel='experiment_all'
            self.assertEqual({f[0] for f in ui.fields()},set(ui.c['experiment']))
    def test_mouse_opens_geometry_and_chart(self):
        from qa_dashboard import Screen
        from unittest.mock import patch
        ui=d.Dashboard(Screen())
        for target in ['geometry',('detail','caster_angle_body_rad')]:
            ui.panel='dashboard';ui.draw()
            hit=next(h for h in ui.hits if h[-1]==target)
            with patch.object(d.curses,'getmouse',return_value=(0,hit[1],hit[0],0,d.curses.BUTTON1_CLICKED)):
                ui.key(d.curses.KEY_MOUSE)
            self.assertEqual(ui.panel,'geometry' if target=='geometry' else 'plot')
    def test_typing_replaces_old_number(self):
        from qa_dashboard import Screen
        from unittest.mock import patch
        screen=Screen();screen.timeout=lambda _:None;screen.move=lambda *_:None
        keys=iter('0.06\n');screen.get_wch=lambda:next(keys)
        ui=d.Dashboard(screen,autoload=False)
        with patch.object(d.curses,'curs_set'):
            self.assertEqual(ui.input_number('Caster trail (m)',.05,'Distance behind swivel axis.'),.06)
    def test_all_panels_draw_and_run_button_is_clickable(self):
        from qa_dashboard import Screen
        for width,height in [(80,24),(125,40)]:
            screen=Screen(width,height);ui=d.Dashboard(screen)
            for panel in ['dashboard','experiment','geometry','physics','initial','controller','plot','data','archive','help']:
                ui.panel=panel;ui.draw()
                self.assertTrue(any(hit[-1]=='run' for hit in ui.hits))
                self.assertTrue(any(hit[-1]=='quit' for hit in ui.hits))
            ui.panel='dashboard';ui.draw()
            for name in ['TRAJECTORY','ANGLES','WHEEL SPEED','ENERGY']:self.assertIn(name,screen.contents())
    def test_model_change_keeps_separate_settings(self):
        from qa_dashboard import Screen
        ui=d.Dashboard(Screen(),autoload=False)
        ui.c['geometry']['caster_trail_m']=.06
        ui.action('v1');self.assertEqual(ui.version,'V1')
        ui.action('v2');self.assertEqual(ui.c['geometry']['caster_trail_m'],.06)

class NavigationTests(unittest.TestCase):
    def make_ui(self):
        from qa_dashboard import Screen
        return d.Dashboard(Screen(),autoload=False)
    def test_tab_reaches_every_setup_action_without_running(self):
        ui=self.make_ui();ui.draw();seen=set()
        for _ in range(50):
            seen.add(ui.nav_action);ui.key(9);ui.draw()
        for action in ['v1','v2','experiment_all','run','help']:
            self.assertIn(action,seen)
        self.assertIsNone(ui.job)
    def test_shift_tab_reverses_tab(self):
        ui=self.make_ui();ui.draw();before=(ui.focus,ui.nav_action,ui.menu_index)
        ui.key(9);ui.draw();ui.key(d.curses.KEY_BTAB);ui.draw()
        self.assertEqual((ui.focus,ui.nav_action,ui.menu_index),before)
    def test_enter_activates_focused_button(self):
        from unittest.mock import patch
        ui=self.make_ui();ui.key(ord('1'));ui.draw();ui.nav_action='run';ui.focus='content'
        with patch.object(ui,'start') as start:ui.key(10);start.assert_called_once()
        self.assertEqual(ui.version,'V1')
    def test_back_restores_previous_page_and_selection(self):
        ui=self.make_ui();ui.action('geometry');ui.draw();ui.field_index=3;ui.nav_action=('field',3)
        ui.action('help');ui.key(27);ui.draw()
        self.assertEqual(ui.panel,'geometry');self.assertEqual(ui.nav_action,('field',3))
        ui.key(27);self.assertEqual(ui.panel,'experiment')
    def test_function_keys_follow_workflow_without_starting_a_run(self):
        ui=self.make_ui()
        for key,panel in [(d.curses.KEY_F3,'dashboard'),(d.curses.KEY_F4,'data'),(d.curses.KEY_F2,'experiment'),(d.curses.KEY_F1,'help')]:
            ui.key(key);self.assertEqual(ui.panel,panel)
        self.assertIsNone(ui.job)
    def test_all_fields_remain_keyboard_reachable(self):
        ui=self.make_ui();ui.action('geometry');ui.draw();seen=set()
        for _ in range(100):
            seen.add(ui.nav_action);ui.key(d.curses.KEY_DOWN);ui.draw()
        for i in range(len(ui.fields())):self.assertIn(('field',i),seen)
    def test_page_down_scrolls_data_and_mouse_wheel_does_not_change_focus(self):
        ui=self.make_ui();ui.action('data');ui.key(d.curses.KEY_NPAGE)
        self.assertGreater(ui.data_offset,0)
    def test_graph_shortcut_keeps_focus_in_graphs_when_already_on_graphs(self):
        ui=self.make_ui();ui.load_latest();ui.panel='dashboard';ui.focus='menu'
        ui.key(d.curses.KEY_F3);ui.key(9)
        self.assertEqual(ui.focus,'content');self.assertEqual(ui.nav_action,('detail','caster_angle_body_rad'))
    def test_letter_shortcuts_match_the_visible_workflow(self):
        ui=self.make_ui()
        for key,panel in [('g','dashboard'),('m','data'),('s','experiment'),('h','help')]:
            ui.key(ord(key));self.assertEqual(ui.panel,panel)
        self.assertIsNone(ui.job)
    def test_completion_does_not_close_the_page_being_read(self):
        from unittest.mock import patch
        from app import Console
        ui=self.make_ui();ui.panel='help';ui.job=object()
        with tempfile.TemporaryDirectory() as tmp:
            ui.run_dir=Path(tmp)
            def finish(instance):instance.job=None;instance.panel='dashboard'
            with patch.object(Console,'poll',finish):ui.poll()
        self.assertEqual(ui.panel,'help')

class QuickKeyboardTests(unittest.TestCase):
    def make_ui(self):
        from qa_dashboard import Screen
        return d.Dashboard(Screen(80,24),autoload=False)
    def test_model_then_enter_runs_without_tab(self):
        from unittest.mock import patch
        ui=self.make_ui()
        for number,version in [('1','V1'),('2','V2')]:
            ui.key(ord(number))
            with patch.object(ui,'start') as start:ui.key(10);start.assert_called_once()
            self.assertEqual(ui.version,version)
    def test_number_keys_open_the_named_sections(self):
        ui=self.make_ui()
        for key,panel in [('3','experiment'),('4','geometry'),('5','physics'),('6','initial'),('7','controller'),('8','data'),('9','archive'),('0','help')]:
            ui.key(ord(key));self.assertEqual(ui.panel,panel)
    def test_letter_edits_visible_field_and_enter_keeps_custom_inputs(self):
        from unittest.mock import patch
        ui=self.make_ui();ui.key(ord('2'))
        with patch.object(ui,'input_number',return_value=.55):ui.key(ord('b'))
        self.assertEqual(ui.c['experiment']['segment_distance_m'],.55)
        with patch.object(ui,'start') as start:ui.key(10);start.assert_called_once()
        self.assertEqual(ui.c['experiment']['segment_distance_m'],.55)
    def test_field_pages_have_six_direct_keys(self):
        from unittest.mock import patch
        ui=self.make_ui();ui.key(ord('5'));ui.draw();first=ui.visible_fields()[1]
        ui.key(ord('n'));ui.draw();start=ui.visible_fields()[1];self.assertGreater(start,first)
        with patch.object(ui,'edit_value') as edit:ui.key(ord('a'));edit.assert_called_once_with(ui.fields()[start])
    def test_next_on_a_plot_opens_results(self):
        ui=self.make_ui();ui.action(('detail','caster_angle_body_rad'));ui.key(10)
        self.assertEqual(ui.panel,'data');ui.key(10);self.assertEqual(ui.panel,'experiment')
