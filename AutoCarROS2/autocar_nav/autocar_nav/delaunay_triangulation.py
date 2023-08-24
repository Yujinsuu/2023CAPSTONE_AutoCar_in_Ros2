#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
import copy

class DelaunayTriPath:
    def __init__(self, cones):
        self.cones = cones
        self.cones_x = cones[:,0]
        self.cones_y = cones[:,1]
        self.cones_color = cones[:,2]

        # 들로네 삼각분할을 시행한 후 저장할 변수들
        self.deltri = np.empty((0,3))
        self.good_deltri = np.empty((0,3))
        self.bad_deltri = np.empty((0,3))
        self.temp_deltri = np.empty((0,3))

        # mid points 저장할 공간
        self.midpoints = np.empty((0,2))
        self.temp_triangle = np.empty((0,3))

        # 들로네 삼각분할 시행한 삼각형들 중
        # 유효한 삼각형들에 대한 라바콘 위치와 색 저장
        self.valid_cones_xycolor = np.empty((0,3))
        self.valid_blue_cones_xy= np.empty((0,2))
        self.valid_yellow_cones_xy = np.empty((0,2))

        self.del_tri_main()
        pass

    def del_tri_main(self):
        # 1. 모든 라바콘에 대해 들로네 삼각분할 시행
        self.delaunay_tri_all_cones()

        # 2. 에러값을 줄이기 위해 같은 색으로 이루어진 들로네 삼각형 삭제
        self.delaunay_tri_find_same_color()

        # 3. good_deltri 중 원점에서 가장 가까운 들로네 삼각형 찾기
        closest_triangle =  self.find_closest_tri()

        # 4. in_track 들로네 삼각형 찾기
        self.get_delaunay_tri_in_track(closest_triangle)

        # 5. midpoint와 valid cones 위치 찾기
        self.get_mid_and_valid_cones_xycolor()

    def delaunay_tri_all_cones(self):
        import matplotlib.tri as mtri

        # 모든 점에 대해서 들로네 삼각분할 시행
        delaunay_triangles = mtri.Triangulation(self.cones_x, self.cones_y)
        self.deltri = delaunay_triangles.get_masked_triangles() # [[a,b,c], [d,e,f], ...]
        pass

    def delaunay_tri_find_same_color(self):
        for t in self.deltri:
            a_ind, b_ind, c_ind = map(int, t[:3])
            a_color = self.cones_color[a_ind]
            b_color = self.cones_color[b_ind]
            c_color = self.cones_color[c_ind]

            # 같은 색으로 이루어진 들로네 삼각형은 bad 들로네 삼각형
            if(a_color == b_color == c_color):
                self.bad_deltri = np.vstack((self.bad_deltri, [t]))

            # 아니면 "우선은" good 들로네 삼각형으로 판단
            else:
                self.good_deltri = np.vstack((self.good_deltri, [t]))
        pass

    def find_closest_tri(self):
        # 시작점에서 제일 가까운 들로네 삼각형 찾기 (삼각형의 중심을 이용)
        closest_dist_from_zero = float('inf')
        closest_triangle = None
        for t in self.good_deltri:
            a_ind, b_ind, c_ind = map(int, t[:3])
            a_color, a_x, a_y = self.cones_color[a_ind], self.cones_x[a_ind], self.cones_y[a_ind]
            b_color, b_x, b_y = self.cones_color[b_ind], self.cones_x[b_ind], self.cones_y[b_ind]
            c_color, c_x, c_y = self.cones_color[c_ind], self.cones_x[c_ind], self.cones_y[c_ind]

            colors = [a_color, b_color, c_color]
            if (colors.count(0) == 2): same_color = 0 # 노란색이 두 개일 때 same_colror = 0
            elif (colors.count(1) == 2): same_color = 1 # 파란색이 두 개일 때 same_color = 1

            # 삼각형 중심
            cx = (a_x + b_x + c_x) / 3
            cy = (a_y + b_y + c_y) / 3
            center_point = np.array([cx, cy])

            # 원점과 중심 사이의 거리 계산
            # 유클리디안 거리
            zero = np.array([-3.0, 0.0])
            distance_from_zero = np.linalg.norm(center_point - zero)

            # 현재까지 찾은 삼각형 중 가장 가까운 삼각형보다 더 가까운 삼각형을 찾으면 업데이트
            if(distance_from_zero < closest_dist_from_zero):
                closest_dist_from_zero = distance_from_zero
                closest_triangle = t

        self.temp_triangle = np.vstack((self.temp_triangle, [closest_triangle]))
        self.good_deltri = np.delete(self.good_deltri, np.where((self.good_deltri == closest_triangle).all(axis=1))[0][0], axis=0)
        #print(closest_triangle)
        return closest_triangle

    def get_delaunay_tri_in_track(self, closest_triangle):
        # 가장 가까운 들로네 삼각형을 시작으로, 맞닿아 있는 선 중 다른 색깔의 점으로만 이루어진 삼각형을 이어준다.
        # 가장 가까운 들로네 삼각형과 인덱스가 두 개 겹치는 삼각형을 찾고, 그 인덱스가 서로 다른 삼각형이면 next_triangle로 지정
        length_ = len(self.good_deltri)
        for i in range(length_):
            for t in self.good_deltri:
                # 겹치는 인덱스 찾기
                com_idxs = [value for value in closest_triangle if value in t]
                # 인덱스 두 개가 겹치면
                if len(com_idxs)==2:
                    com_idx_1, com_idx_2= (int)(com_idxs[0]) , (int)(com_idxs[1])
                    # 두 인덱스의 색깔이 다르면 다음 삼각형으로 취급
                    if(self.cones_color[com_idx_1] != self.cones_color[com_idx_2]):
                        self.temp_triangle = np.vstack((self.temp_triangle, [t]))
                        self.good_deltri = np.delete(self.good_deltri, np.where((self.good_deltri == t).all(axis=1))[0][0], axis=0)
                        closest_triangle = t
                        break
        self.good_deltri = self.temp_triangle
        pass

    def get_mid_and_valid_cones_xycolor(self):
        for t in self.good_deltri:
            # good_deltri의 세 라바콘의 색과 위치 받아오기
            a_ind, b_ind, c_ind = map(int, t[:3])
            a_color, a_x, a_y = self.cones_color[a_ind], self.cones_x[a_ind], self.cones_y[a_ind]
            b_color, b_x, b_y = self.cones_color[b_ind], self.cones_x[b_ind], self.cones_y[b_ind]
            c_color, c_x, c_y = self.cones_color[c_ind], self.cones_x[c_ind], self.cones_y[c_ind]

            # midpoint 계산
            if(a_color != b_color):
                self._get_cones_mid_(a_ind, b_ind)
            if(b_color != c_color):
                self._get_cones_mid_(b_ind, c_ind)
            if(c_color != a_color):
                self._get_cones_mid_(c_ind, a_ind)

            def update_valid_cones_xy(color, x, y):
                if color == 0:
                    self.valid_blue_cones_xy = np.vstack((self.valid_blue_cones_xy, [x, y]))
                elif color == 1:
                    self.valid_yellow_cones_xy = np.vstack((self.valid_yellow_cones_xy, [x, y]))
            update_valid_cones_xy(a_color, a_x, a_y)
            update_valid_cones_xy(b_color, b_x, b_y)
            update_valid_cones_xy(c_color, c_x, c_y)

        #원점에서 제일 가까운 점, 그리고 그 점에서 제일 가까운 점 ...
        temp_point = [-2.0, 0.0]
        temp_mids = copy.deepcopy(self.midpoints)
        sorted_mids = np.empty((0,2))

        for i in range(len(self.midpoints)):
            closest_point = self._find_closest_point_(temp_point, temp_mids) #temp_mids 안에 았는 node 중 temp_node와 제일 가까운 점 찾기
            temp_mids = np.delete(temp_mids, np.where((temp_mids == temp_mids).all(axis=1))[0][0], axis=0)
            sorted_mids = np.vstack((sorted_mids, closest_point))
            temp_point = closest_point

        self.midpoints = sorted_mids.copy()
        self.valid_blue_cones_xy = np.unique(self.valid_blue_cones_xy, axis=0)
        self.valid_yellow_cones_xy = np.unique(self.valid_yellow_cones_xy, axis=0)
        pass

    def get_mid(self):
        return self.midpoints

    def get_blue(self):
        return self.valid_blue_cones_xy

    def get_yellow(self):
        return self.valid_yellow_cones_xy

    # midpoint 계산하는 내부 함수
    def _get_cones_mid_(self, idx1, idx2):
            midpoint_x = round((self.cones_x[idx1] + self.cones_x[idx2])/2, 2)
            midpoint_y = round((self.cones_y[idx1] + self.cones_y[idx2])/2, 2)

            # 중복 체크
            if not np.any(np.all(self.midpoints == [midpoint_x, midpoint_y], axis=1)):
                self.midpoints = np.vstack((self.midpoints, [midpoint_x, midpoint_y]))

    # 해당 점에서 제일 가까운 점 찾는 내부 함수
    def _find_closest_point_(self, p, points):
        min_distance = float('inf')  # 초기 최소 거리를 무한대로 설정
        closest_point = None  # 초기 가장 가까운 점을 None으로 설정

        for point in points:
            distance = np.linalg.norm(p - point)  # p와 각 점의 거리 계산

            if distance < min_distance:
                min_distance = distance
                closest_point = point

        return closest_point
