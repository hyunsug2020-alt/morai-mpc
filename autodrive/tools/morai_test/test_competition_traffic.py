import copy
import unittest

import competition_traffic as traffic


class RegionalTrafficTest(unittest.TestCase):
    def scene(self, count):
        links = ['A2256W000133', 'A2256W000134', 'A2256W000135',
                 'A2256W000411', 'A2256W000409', 'A2256W000420',
                 'A2256W000126', 'A2256W000126', 'A2256W000136',
                 'A2256W000411', 'A2256W000420', 'A2256W000410',
                 'A2256W000431', 'A2256W000435']
        return dict(vehicleList=[dict(UNIQUEID=20000+i, initPosition=dict(initLink=link))
                                 for i, link in enumerate(links[:count])], spawnPointList=[])

    def test_current_fourteen_vehicles_and_archived_scenes_are_admitted(self):
        for count in [8, 11, 14]:
            scene = self.scene(count)
            self.assertTrue(traffic.recognized(scene))
            ids = {str(v['UNIQUEID']) for v in scene['vehicleList']}
            groups = {zone: group+[uid for uid in traffic.ADDED[zone] if uid in ids]
                      for zone, (_, group) in traffic.ZONES.items()}
            self.assertEqual({uid for group in groups.values() for uid in group}, ids)
            if count == 14:
                self.assertEqual({zone: len(group) for zone, group in groups.items()},
                                 dict(roundabout=4, highway=8, tunnel=2))

    def test_missing_extra_vehicle_wrong_native_start_and_spawners_are_rejected(self):
        scene = self.scene(14)
        missing = copy.deepcopy(scene)
        missing['vehicleList'].pop()
        self.assertFalse(traffic.recognized(missing))
        wrong = copy.deepcopy(scene)
        wrong['vehicleList'][-1]['initPosition']['initLink'] = 'A2256W000126'
        self.assertFalse(traffic.recognized(wrong))
        scene['spawnPointList'] = [dict(isSpawning=True)]
        self.assertFalse(traffic.recognized(scene))


if __name__ == '__main__':
    unittest.main()
