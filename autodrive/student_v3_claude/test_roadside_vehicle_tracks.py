"""A reclassified model car must not survive as a dropout forecast."""
import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location('copy_vehicle_detection',
    Path(__file__).parent/'lidar_vehicle_detection.py')
vehicle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vehicle)


class RoadsideTrackTests(unittest.TestCase):
    def detection(self, x=-101.6):
        return dict(position=[x, 319.5], local_position=[2., 2.8], local_z=.9,
                    size=[5., 2.1, 1.5], heading_deg=277., score=.6)

    def test_confirmed_structure_retires_the_matching_false_car_forecast(self):
        tracker = vehicle.VehicleTracks()
        detection = self.detection()
        uid = tracker.update([detection], 100.)[0]['uid']
        retired = tracker.reclassify_roadside([dict(detection, roadside_structure='rail')], [])
        self.assertEqual(retired, [uid])
        self.assertEqual(tracker.update([], 100.1), [])

    def test_neighbor_and_new_real_vehicle_measurements_keep_their_tracks(self):
        tracker = vehicle.VehicleTracks()
        fake, real = self.detection(), self.detection(-100.)
        tracks = tracker.update([fake, real], 100.)
        retired = tracker.reclassify_roadside([fake], [real])
        self.assertEqual(retired, [tracks[0]['uid']])
        self.assertIn(tracks[1]['uid'], tracker.tracks)
        tracker = vehicle.VehicleTracks()
        uid = tracker.update([fake], 100.)[0]['uid']
        actual = self.detection(-101.5)
        self.assertEqual(tracker.reclassify_roadside([fake], [actual]), [])
        self.assertIn(uid, tracker.tracks)

    def test_incompatible_shape_cannot_retire_a_nearby_car(self):
        tracker = vehicle.VehicleTracks()
        detection = self.detection()
        uid = tracker.update([detection], 100.)[0]['uid']
        for changed in (dict(heading_deg=320.), dict(size=[8., 3., 1.5]), dict(position=[-102.2, 319.5])):
            self.assertEqual(tracker.reclassify_roadside([dict(detection, **changed)], []), [])
            self.assertIn(uid, tracker.tracks)


if __name__ == '__main__':
    unittest.main()
