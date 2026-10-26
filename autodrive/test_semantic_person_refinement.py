"""Recorded red-post correction must preserve actual people and uncertain input."""
import ast
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'semantic/pipeline'))
from semantic_person_refinement import (small_person_component, context_rois, merge_contexts,
                                       recover_person_contexts, forward_person_rois)

DATA = ROOT/'autodrive/student_v3_claude/test_data/static_person_refinement_20261023.npz'
FORWARD_DATA = ROOT/'autodrive/student_v3_claude/test_data/person_forward_contexts_20261023.npz'


def placement(values):
    return dict(zip(('left', 'top', 'width', 'height'), map(int, values)))


def recording(tag, path=DATA):
    with np.load(path, allow_pickle=False) as data:
        original = data[tag+'_base'].copy()
        place = placement(data[tag+'_place'])
        shape = tuple(data[tag+'_shape'])
        component = small_person_component(original, place, shape)
        contexts = [dict(mask=data[tag+'_mask_'+str(i)].copy(),
                         placement=placement(data[tag+'_place_'+str(i)]),
                         roi=tuple(data[tag+'_roi_'+str(i)])) for i in range(2)]
        jpeg = data[tag+'_jpeg'].copy()
    return original, place, component, contexts, jpeg


class SmallPersonRefinementTests(unittest.TestCase):
    def test_distant_actual_person_is_recovered_from_both_recorded_contexts(self):
        original, place, component, contexts, _ = recording('people', FORWARD_DATA)
        missing = original.copy()
        missing[component['yy'], component['xx']] = 255
        recovered, evidence = recover_person_contexts(missing, place,
            component['image_shape'], contexts)
        self.assertGreater(evidence['added_person_pixels'], 100)
        self.assertGreater(np.count_nonzero(recovered[component['yy'], component['xx']] == 1), 100)
        np.testing.assert_array_equal(recovered[missing == 0], missing[missing == 0])
        np.testing.assert_array_equal(recovered[missing == 254], missing[missing == 254])
        self.assertTrue(np.all(missing[component['yy'], component['xx']] == 255))

    def test_recorded_red_post_does_not_gain_a_recovered_person(self):
        original, place, component, contexts, _ = recording('post', FORWARD_DATA)
        original = np.full_like(original, 255)
        result, evidence = recover_person_contexts(original, place,
            component['image_shape'], contexts)
        self.assertIs(result, original)
        self.assertEqual(evidence['added_person_pixels'], 0)

    def test_missing_conflicting_or_duplicate_person_context_cannot_add_a_person(self):
        for change in ('missing', 'unknown', 'duplicate'):
            original, place, component, contexts, _ = recording('people')
            original = np.full_like(original, 255)
            if change == 'missing':
                contexts = contexts[:1]
            elif change == 'unknown':
                contexts[1]['mask'][:] = 255
            else:
                contexts[1] = contexts[0]
            with self.subTest(change=change):
                result, evidence = recover_person_contexts(original, place,
                    component['image_shape'], contexts)
                self.assertIs(result, original)
                self.assertEqual(evidence['added_person_pixels'], 0)

    def test_person_recovery_preserves_vehicles_people_blind_pixels_and_confirmed_posts(self):
        _, place, component, contexts, _ = recording('people')
        for kind in (0, 1, 254):
            original = np.full_like(contexts[0]['mask'], kind)
            result, evidence = recover_person_contexts(original, place,
                component['image_shape'], contexts)
            self.assertIs(result, original)
            self.assertEqual(evidence['added_person_pixels'], 0)
        original = np.full_like(contexts[0]['mask'], 2)
        result, evidence = recover_person_contexts(original, place,
            component['image_shape'], contexts, protected=np.ones_like(original, bool))
        self.assertIs(result, original)
        self.assertEqual(evidence['added_person_pixels'], 0)
        rois = forward_person_rois((720, 1280))
        self.assertEqual(len(set(rois)), 2)
        self.assertTrue(all(x >= 0 and y >= 0 and x+w <= 1280 and y+h <= 720
                            for x, y, w, h in rois))

    def test_near_complete_component_gets_context_before_larger_far_noise(self):
        mask = np.full((100, 100), 255, np.uint8)
        mask[15:25, 40:45] = 1
        mask[65:70, 35:38] = 1
        mask[98:100, 10:20] = 1  # clipped at the camera edge
        component = small_person_component(mask,
            dict(left=0, top=0, width=100, height=100), (100, 100))
        self.assertEqual(len(component['xx']), 15)
        self.assertGreater(float(component['source_xy'][:, 1].min()), 60.)

    def test_recorded_post_becomes_a_physical_obstacle_without_changing_other_pixels(self):
        original, _, component, contexts, _ = recording('post')
        before = original.copy()
        result, evidence = merge_contexts(original, component, contexts)
        self.assertEqual(evidence['relabelled_pixels'], 114)
        self.assertTrue(np.all(result[component['yy'], component['xx']] == 2))
        expected = before.copy()
        expected[component['yy'], component['xx']] = 2
        np.testing.assert_array_equal(result, expected)
        np.testing.assert_array_equal(original, before)

    def test_recorded_actual_person_survives_both_current_contexts(self):
        original, _, component, contexts, _ = recording('people')
        self.assertGreaterEqual(len(component['xx']), 100)
        result, evidence = merge_contexts(original, component, contexts)
        self.assertIs(result, original)
        self.assertEqual(evidence['relabelled_pixels'], 0)
        self.assertTrue(np.all(result[component['yy'], component['xx']] == 1))

    def test_person_support_unknown_context_missing_scale_and_clipping_keep_original(self):
        for condition in ('person', 'unknown', 'one_scale', 'duplicate', 'clipped'):
            original, _, component, contexts, _ = recording('post')
            if condition in ('person', 'unknown'):
                contexts[1]['mask'][:] = 1 if condition == 'person' else 255
            elif condition == 'one_scale':
                contexts = contexts[:1]
            elif condition == 'duplicate':
                contexts[1] = contexts[0]
            else:
                contexts[1]['mask'] = np.zeros((1, 1), np.uint8)
            with self.subTest(condition=condition):
                result, evidence = merge_contexts(original, component, contexts)
                self.assertIs(result, original)
                self.assertEqual(evidence['relabelled_pixels'], 0)

    def sidecar(self, infer, function_name='refine_front_person'):
        # Import only the integration function, without starting the CLI/GPU loop.
        tree = ast.parse((ROOT/'autodrive/semantic_sidecar.py').read_text())
        node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == function_name)
        from semantic_common import letterbox
        namespace = dict(cv2=cv2, np=np, small_person_component=small_person_component,
            context_rois=context_rois, merge_contexts=merge_contexts, letterbox=letterbox,
            forward_person_rois=forward_person_rois, recover_person_contexts=recover_person_contexts,
            width=640, height=384, exclusion_for=lambda *args: None,
            MEAN=np.array([.485, .456, .406], np.float32)*255.,
            STD=np.array([.229, .224, .225], np.float32)*255.,
            refinement_gpu=SimpleNamespace(run=infer))
        exec(compile(ast.Module(body=[node], type_ignores=[]), 'semantic_sidecar.py', 'exec'), namespace)
        return namespace[function_name]

    def test_sidecar_recovers_a_missed_person_without_changing_the_source_frame(self):
        original, place, component, contexts, jpeg = recording('people', FORWARD_DATA)
        original[component['yy'], component['xx']] = 255
        frames = dict(front_jpeg=jpeg, front_stamp_ns=123456789)
        item = dict(frames=frames, images=[cv2.imdecode(jpeg, cv2.IMREAD_COLOR)],
                    prepared=[(None, place)])
        batches = []
        def infer(batch):
            batches.append(batch.shape)
            return [c['mask'] for c in contexts]
        masks = [original]
        result = self.sidecar(infer, 'recover_front_people')(item, masks)
        self.assertGreater(result['added_person_pixels'], 100)
        self.assertEqual(batches, [(2, 3, 384, 640)])
        self.assertEqual(frames['front_stamp_ns'], 123456789)
        np.testing.assert_array_equal(frames['front_jpeg'], jpeg)

    def test_sidecar_uses_the_same_jpeg_in_one_two_context_batch_and_keeps_source_stamp(self):
        original, place, _, contexts, jpeg = recording('post')
        frames = dict(front_jpeg=jpeg, front_stamp_ns=123456789)
        item = dict(frames=frames, images=[cv2.imdecode(jpeg, cv2.IMREAD_REDUCED_COLOR_2)],
                    prepared=[(None, place)])
        batches = []
        def infer(batch):
            batches.append(batch.shape)
            return [c['mask'] for c in contexts]
        masks = [original]
        result = self.sidecar(infer)(item, masks)
        self.assertEqual(result['relabelled_pixels'], 114)
        self.assertEqual(batches, [(2, 3, 384, 640)])
        self.assertEqual(frames['front_stamp_ns'], 123456789)
        np.testing.assert_array_equal(frames['front_jpeg'], jpeg)

    def test_extra_inference_failure_cannot_partially_relabel_a_person(self):
        original, place, _, _, jpeg = recording('post')
        before = original.copy()
        item = dict(frames=dict(front_jpeg=jpeg),
            images=[cv2.imdecode(jpeg, cv2.IMREAD_REDUCED_COLOR_2)], prepared=[(None, place)])
        def failed(batch):
            raise RuntimeError('extra observation unavailable')
        masks = [original]
        with self.assertRaisesRegex(RuntimeError, 'extra observation unavailable'):
            self.sidecar(failed)(item, masks)
        self.assertIs(masks[0], original)
        np.testing.assert_array_equal(masks[0], before)


if __name__ == '__main__':
    unittest.main()
