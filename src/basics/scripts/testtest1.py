#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")

        # === 토픽: 그대로 사용 ===
        self.cam_topic    = rospy.get_param("~cam_topic", "/image_jpeg/compressed")
        self.center_topic = rospy.get_param("~center_topic", "/driving_center")
        rospy.Subscriber(self.cam_topic, CompressedImage, self.cam_CB)
        self.center_pub = rospy.Publisher(self.center_topic, Int32, queue_size=3)

        # === 동작 파라미터 ===
        self.fixed_center = rospy.get_param("~fixed_center", 320)   # 640px 가정
        self.show_debug   = rospy.get_param("~show_debug", True)
        self.use_bitpack  = rospy.get_param("~use_bitpack", False)  # True면 비트패킹해서 보냄

        # 가이드 프레임 게이트 & 스무딩
        self.N_HIT        = rospy.get_param("~guide_frames", 2)     # 연속 허용 프레임 수
        self.slope_guard_min = rospy.get_param("~slope_guard_min", -85.0)  # deg
        self.slope_guard_max = rospy.get_param("~slope_guard_max", -25.0)  # deg
        self.ema_alpha_center = rospy.get_param("~ema_alpha_center", 0.7)
        self.ema_alpha_slope  = rospy.get_param("~ema_alpha_slope", 0.7)

        # HSV 범위 (노란/흰 차선)
        self.yellow_lower = np.array([15, 128,  0],  dtype=np.uint8)
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        self.white_lower  = np.array([0,   0, 192],  dtype=np.uint8)
        self.white_upper  = np.array([179, 64, 255], dtype=np.uint8)

        # 점선 판정 파라미터(현장 튜닝 가능)
        self.dash_roi_start_ratio = rospy.get_param("~dash_roi_start_ratio", 0.55)  # 오른쪽 45%
        self.dash_band_half       = rospy.get_param("~dash_band_half", 24)         # ±픽셀
        self.dash_colsum_min      = rospy.get_param("~dash_colsum_min", 400)       # 신호 최소
        self.dash_min_segments    = rospy.get_param("~dash_min_segments", 3)
        self.dash_med_on_max      = rospy.get_param("~dash_med_on_max", 35.0)
        self.dash_on_ratio_min    = rospy.get_param("~dash_on_ratio_min", 0.15)
        self.dash_on_ratio_max    = rospy.get_param("~dash_on_ratio_max", 0.85)

        self.bridge = CvBridge()
        self.img_h, self.img_w = 480, 640  # 기본값(콜백에서 갱신)
        rospy.Timer(rospy.Duration(1.0/10), self.timerCB)

        # 상태
        self.center_ema = None
        self.slope_ema  = None
        self.guide_hits = 0

    # ========================= 메인 콜백 =========================
    def cam_CB(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)  # BGR
        self.img_h, self.img_w = img.shape[:2]

        # 1) 색 필터
        filtered_img = self.hsv(img)

        # 2) 투시변환(BEV 유사)
        warped_img = self.perspective_transform(filtered_img, self.img_w, self.img_h)

        # 3) 바이너리
        bin_img = self.binary(warped_img)

        # 4) 가이드 우선, 실패 시 히스토그램
        guide_found, guide_center, slope_deg, dbg = self.right_guideline_target(bin_img)
        if guide_found:
            self.guide_hits = min(self.guide_hits + 1, self.N_HIT)
        else:
            self.guide_hits = 0

        if self.guide_hits >= self.N_HIT:
            # 기울기 가드(비정상 각도면 폴백)
            if self.slope_ema is None:
                use_slope = slope_deg
            else:
                use_slope = self.slope_ema

            if self.slope_guard_min <= use_slope <= self.slope_guard_max:
                center_index = int(guide_center)
            else:
                left_idx, right_idx = self.histogram(bin_img)
                center_index = self.judgement_hist(left_idx, right_idx)
        else:
            left_idx, right_idx = self.histogram(bin_img)
            center_index = self.judgement_hist(left_idx, right_idx)

        # 5) 노란 점선 여부(선택) — warped에서 다시 HSV로
        yellow_dashed = self.detect_yellow_dashed(warped_img)

        # 6) 가드 & 범위 보정
        if center_index is None or not np.isfinite(center_index):
            center_index = self.fixed_center
        center_index = int(np.clip(center_index, 0, self.img_w - 1))

        # 7) EMA 스무딩
        center_sm = int(round(self.ema(self.center_ema, center_index, self.ema_alpha_center)))
        self.center_ema = center_sm
        slope_sm  = float(self.ema(self.slope_ema, slope_deg, self.ema_alpha_slope)) if guide_found else (self.slope_ema if self.slope_ema is not None else 0.0)
        self.slope_ema = slope_sm

        # 로그
        if dbg is not None:
            rospy.loginfo(f"[GUIDE DBG] roi_nz={dbg['roi_nz']}, edge_nz={dbg['edge_nz']}, hough={dbg['hough']}, kept={dbg['kept']}, rmse={dbg['rmse']:.1f}, x_pred={dbg['x_pred']}, slope={slope_deg:.1f}")
        rospy.loginfo(f"[GUIDE] found={guide_found}, guide_center={guide_center}, slope={slope_deg:.1f}deg, hits={self.guide_hits}/{self.N_HIT}")
        rospy.loginfo(f"[CENTER] final center_index={center_sm}, dashed={yellow_dashed}")

        # 8) 퍼블리시 (기본: center만, 옵션: 비트패킹)
        if not self.use_bitpack:
            self.publish(center_sm)
        else:
            packed = self.pack_bits(center_sm, yellow_dashed, slope_sm)
            self.publish(packed)

        # 디버그 뷰
        if self.show_debug:
            try:
                #cv2.imshow("Original", img)
                #cv2.imshow("Warped", warped_img)
                cv2.imshow("Binary", bin_img)
                #if dbg and 'roi' in dbg:
                    #cv2.imshow("ROI", dbg['roi'])
                #if dbg and 'bridged' in dbg:
                    #cv2.imshow("Bridged", dbg['bridged'])
                #if dbg and 'edges' in dbg:
                    #cv2.imshow("Edges", dbg['edges'])
                cv2.waitKey(1)
            except:
                pass

    # ========================= 유틸 =========================
    def ema(self, prev, new, alpha=0.7):
        return new if prev is None else (alpha*prev + (1.0-alpha)*new)

    # ========== 우측 가이드(점선) 기반 타겟 산출 ==========
    def right_guideline_target(self, bin_warped):
        H, W = bin_warped.shape

        # ROI 다각형(넓게) + 디버그 저장
        roi_mask = np.zeros_like(bin_warped)
        poly = np.array([
            [int(0.50*W), int(0.50*H)],
            [int(0.98*W), int(0.50*H)],
            [int(0.98*W), int(0.98*H)],
            [int(0.55*W), int(0.98*H)],
        ], dtype=np.int32)
        cv2.fillPoly(roi_mask, [poly], 255)
        roi = cv2.bitwise_and(bin_warped, roi_mask)
        roi_nonzero = int(np.count_nonzero(roi))

        # 점선 브리징(CLOSE)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 31))
        bridged = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Canny & Hough (완화 파라미터)
        edges = cv2.Canny(bridged, 40, 120, apertureSize=3, L2gradient=True)
        edge_cnt = int(np.count_nonzero(edges))
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=18,
                                minLineLength=10, maxLineGap=30)
        total_lines = 0 if lines is None else len(lines)

        sel_lines = []
        if lines is not None:
            for (x1, y1, x2, y2) in lines.reshape(-1, 4):
                dx, dy = (x2 - x1), (y2 - y1)
                if dx == 0:
                    continue
                slope = dy / float(dx)
                # 우측 대역 & 충분한 기울기 (부호 고정X)
                if (x1 > 0.55*W or x2 > 0.55*W) and (abs(slope) > 0.2):
                    sel_lines.append((x1, y1, x2, y2))

        kept_lines = len(sel_lines)
        if kept_lines == 0:
            dbg = dict(roi=roi, bridged=bridged, edges=edges,
                       roi_nz=roi_nonzero, edge_nz=edge_cnt, hough=total_lines, kept=0, rmse=999.9, x_pred=-1)
            return False, None, 0.0, dbg

        # 피팅 (x = a*y^2 + b*y + c)
        pts = []
        for (x1, y1, x2, y2) in sel_lines:
            pts.append((x1, y1)); pts.append((x2, y2))
        pts = np.asarray(pts, dtype=np.float32)
        ys, xs = pts[:, 1], pts[:, 0]

        try:
            coeffs = np.polyfit(ys, xs, 2)
        except Exception:
            dbg = dict(roi=roi, bridged=bridged, edges=edges,
                       roi_nz=roi_nonzero, edge_nz=edge_cnt, hough=total_lines, kept=kept_lines, rmse=999.9, x_pred=-1)
            return False, None, 0.0, dbg

        x_fit = np.polyval(coeffs, ys)
        rmse = float(np.sqrt(np.mean((xs - x_fit) ** 2)))
        if rmse > 20.0:
            dbg = dict(roi=roi, bridged=bridged, edges=edges,
                       roi_nz=roi_nonzero, edge_nz=edge_cnt, hough=total_lines, kept=kept_lines, rmse=rmse, x_pred=-1)
            return False, None, 0.0, dbg

        y_look = int(0.65 * H)
        x_pred = int(np.clip(np.polyval(coeffs, y_look), 0, W - 1))

        # 기울기(도) 계산 (영상 좌표 y-아래로 증가 → -dy)
        y0 = max(0, y_look - 10); y1p = min(H - 1, y_look + 10)
        x0 = float(np.polyval(coeffs, y0)); x1v = float(np.polyval(coeffs, y1p))
        dx, dy = (x1v - x0), (y1p - y0)
        slope_deg = float(np.degrees(np.arctan2(-dy, dx)))

        dbg = dict(roi=roi, bridged=bridged, edges=edges,
                   roi_nz=roi_nonzero, edge_nz=edge_cnt, hough=total_lines, kept=kept_lines, rmse=rmse, x_pred=x_pred)
        return True, x_pred, slope_deg, dbg

    # ========== 히스토그램 폴백 ==========
    def histogram(self, bin_img):
        hist = np.sum(bin_img, axis=0)  # 열 합
        left_hist  = hist[:self.fixed_center]
        right_hist = hist[self.fixed_center:]
        left_idx  = np.where(left_hist  > 5)[0]
        right_idx = np.where(right_hist > 5)[0] + self.fixed_center
        return left_idx, right_idx

    def judgement_hist(self, left_idx, right_idx):
        center_index = self.fixed_center
        if len(left_idx) > 0 and len(right_idx) > 0:
            if len(left_idx) > 70:
                center_index = int((0.5 * right_idx[-1]) + 40)
            elif len(right_idx) > 60:
                center_index = int((0.5 * right_idx[-1]) + 40)
            else:
                center_index = int((left_idx[0] + right_idx[-1]) // 2)
            return center_index
        if len(right_idx) > 0 and len(left_idx) == 0:
            return int((0.5 * right_idx[-1]) + 40)
        if len(left_idx) > 0 and len(right_idx) == 0:
            return int((left_idx[0] + self.fixed_center) // 2)
        return center_index

    # ========== 색 필터 / 투시 / 이진 ==========
    def hsv(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        m_w = cv2.inRange(hsv, self.white_lower,  self.white_upper)
        mask = cv2.bitwise_or(m_y, m_w)
        return cv2.bitwise_and(bgr, bgr, mask=mask)

    def perspective_transform(self, bgr, W, H):
        # 640x480 기준 좌표 → 현재 해상도로 스케일링
        src = np.float32([[0,   460], [225, 315], [415, 315], [640, 460]])
        dst = np.float32([[82,  480], [118,   0], [540,   0], [575, 480]])
        sx = W / 640.0; sy = H / 480.0
        src *= np.array([sx, sy], dtype=np.float32)
        dst *= np.array([sx, sy], dtype=np.float32)
        M   = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(bgr, M, (W, H))

    def binary(self, bgr_warped):
        gray = cv2.cvtColor(bgr_warped, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray)
        bin_img[gray > 50] = 255
        return bin_img

    # ========== 노란 점선 판별(우측 대역, 세로 프로파일 런-길이) ==========
    def detect_yellow_dashed(self, bgr_warped):
        H, W = bgr_warped.shape[:2]
        hsv = cv2.cvtColor(bgr_warped, cv2.COLOR_BGR2HSV)

        m1 = cv2.inRange(hsv, (15,  70, 150), (40, 255, 255))
        m2 = cv2.inRange(hsv, (10,  60, 140), (45, 255, 255))
        m_y = cv2.bitwise_or(m1, m2)
        m_y = cv2.morphologyEx(m_y, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)

        x0 = int(W * self.dash_roi_start_ratio)
        roi = m_y[:, x0:]
        col_sum = roi.sum(axis=0)
        if col_sum.max() < self.dash_colsum_min:
            return False

        peak = int(np.argmax(col_sum))
        half = int(self.dash_band_half)
        band = roi[:, max(0, peak - half): min(roi.shape[1], peak + half + 1)]

        y_profile = band.mean(axis=1).astype(np.uint8)
        _, bin1d = cv2.threshold(y_profile, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        bin1d = (bin1d > 0).astype(np.uint8)

        runs, c = [], 0
        for v in bin1d:
            if v: c += 1
            elif c > 0:
                runs.append(c); c = 0
        if c > 0: runs.append(c)

        if len(runs) == 0:
            return False

        med = float(np.median(runs))
        on_ratio = float(bin1d.mean())
        return (len(runs) >= self.dash_min_segments) and \
               (med <= self.dash_med_on_max) and \
               (self.dash_on_ratio_min <= on_ratio <= self.dash_on_ratio_max)

    # ========== 퍼블리시 ==========
    def publish(self, value):
        msg = Int32()
        msg.data = int(value)
        self.center_pub.publish(msg)

    # ========== 비트패킹(옵션) ==========
    # layout:
    #  bits  0..10 : center (0..2047)
    #  bit      11 : yellow_dashed (0/1)
    #  bits 12..18 : slope_deg quantized [-63..+63] -> 7bit (2deg/LSB)
    def pack_bits(self, center, yellow_dashed, slope_deg):
        c = int(np.clip(center, 0, 2047)) & 0x7FF
        d = 1 if yellow_dashed else 0
        q = int(np.clip(round(slope_deg/2.0), -63, 63)) & 0x7F
        return (c | (d << 11) | (q << 12))

    def timerCB(self, _):
        pass

def main():
    try:
        Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
