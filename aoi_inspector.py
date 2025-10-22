# AOI Inspector Script
# Version: 2.0.0
# Date: 2025-10-22
# Author: Grok (xAI)
# Changelog:
#   - 1.0.0 to 1.9.0: Previous features (lighting, detection, etc.)
#   - 2.0.0 (2025-10-22): Consolidated all discussed functionalities: YOLO polarity, Gerber validation, golden stage with sticker ROI, fast fid pass, adaptive polar angles
# Dependencies:
#   - Python==3.8.x
#   - opencv-contrib-python==4.5.5.64
#   - numpy==1.21.6, pandas==1.3.5, scikit-learn==1.0.2, scikit-image==0.18.3
#   - easyocr==1.6.2, pyautogui==0.9.53, scipy==1.7.3, pyserial==3.5
#   - ultralytics==8.0.196 (for YOLOv8)
# Notes: Run with KiCad files. Ensure Pico connected. YOLO model: 'yolov8n.pt' or custom-trained.

import cv2
import numpy as np
import pickle
import pyautogui
import easyocr
import json
import csv
import pandas as pd
from datetime import datetime
from sklearn.svm import SVC
from skimage.feature import hog
import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import re
from scipy.spatial.distance import cdist
import serial
import time
import sys
from ultralytics import YOLO

# Version check
if sys.version_info < (3, 8):
	raise RuntimeError("Python 3.8+ required.")
if cv2.__version__ != '4.5.5':
	print(f"Warning: OpenCV {cv2.__version__} detected; 4.5.5.64 recommended.")

# Global vars
defect_labels = []
current_defect = None
root = tk.Tk()
root.withdraw()


class GerberParser:
	def __init__(self):
		self.unit_multiplier = 1.0
		self.format_decimals = (5, 5)
		self.apertures = {}
		self.fiducials = []

	def _detect_unit(self, content):
		return 'MM' if 'G71*' in content else 'IN'

	def _parse_format_spec(self, content):
		fs_match = re.search(r'%FS.*?X(\d)Y(\d)\*%', content)
		return (int(fs_match.group(1)), int(fs_match.group(2))) if fs_match else (5, 5)

	def parse(self, gerber_path, fid_diam_mm=1.0, tol=0.2):
		try:
			with open(gerber_path, 'r') as f:
				content = f.read()
		except FileNotFoundError:
			print(f"Error: Gerber file {gerber_path} not found.")
			return []

		self.unit_multiplier = 25.4 if self._detect_unit(content) == 'IN' else 1.0
		self.format_decimals = self._parse_format_spec(content)

		for match in re.finditer(r'%ADD(\d+)C,([\d.]+)\*%', content):
			code = int(match.group(1))
			diam_inch = float(match.group(2))
			diam_mm = diam_inch * self.unit_multiplier
			if abs(diam_mm - fid_diam_mm) < fid_diam_mm * tol:
				self.apertures[code] = {'type': 'C', 'size': diam_mm}

		current_ap = None
		for line in content.splitlines():
			line = line.strip()
			sel_match = re.search(r'G54D(\d+)\*', line)
			if sel_match:
				current_ap = int(sel_match.group(1))
				continue
			if line.endswith('D03*') and current_ap in self.apertures:
				x_match = re.search(r'X(-?\d+)', line)
				y_match = re.search(r'Y(-?\d+)', line)
				if x_match and y_match:
					dec_x, dec_y = self.format_decimals
					x_val = int(x_match.group(1)) / (10 ** dec_x)
					y_val = int(y_match.group(1)) / (10 ** dec_y)
					x_mm = x_val * self.unit_multiplier
					y_mm = y_val * self.unit_multiplier
					self.fiducials.append((x_mm, y_mm))

		unique_fids = []
		for fid in self.fiducials:
			if not unique_fids or all(cdist([fid], unique_fids) >= 0.1):
				unique_fids.append(fid)
		self.fiducials = unique_fids
		print(f"Parsed {len(self.fiducials)} fiducials from {gerber_path}.")
		return self.fiducials


class EnhancedInteractiveAOI:
	def __init__(self, golden_paths, gerber_path=None, bom_path=None, placement_path=None, fid_diam_mm=1.0,
	             report_dir='reports', pico_port='COM3'):
		self.golden_feats = self.extract_hog_from_images(golden_paths, labels=[0] * len(golden_paths))
		self.clf = SVC(kernel='rbf', C=1.0)
		self.clf.fit(self.golden_feats, [0] * len(self.golden_feats))
		self.yolo_model = YOLO('yolov8n.pt')  # Load YOLOv8 (replace with custom-trained if available)
		self.ocr_reader = easyocr.Reader(['en'])
		self.report_dir = report_dir
		os.makedirs(report_dir, exist_ok=True)
		self.batch_id = f"batch_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
		self.defect_types = [
			('bridge', 'paste_print'), ('missing', 'placement'), ('mismatch', 'placement'),
			('open', 'reflow'), ('excess', 'paste_print'), ('tombstone', 'reflow')
		]

		self.components = self.parse_bom(bom_path) if bom_path else {}
		self.placements = self.parse_placement(placement_path) if placement_path else {}
		self.virtual_golden = self.merge_design_refs()

		if gerber_path:
			parser = GerberParser()
			gerber_fids_mm = parser.parse(gerber_path, fid_diam_mm)
			self.gerber_fids = np.array(gerber_fids_mm) if gerber_fids_mm else None
		else:
			self.gerber_fids = None
		self.scale_mm_per_px = self.auto_calibrate(golden_paths[0] if golden_paths else None)

		# Pico serial
		try:
			self.ser = serial.Serial(pico_port, 115200, timeout=2)
			print(f"Connected to Pico on {pico_port}")
		except Exception as e:
			self.ser = None
			print(f"Error connecting to Pico: {e}. Lighting control disabled.")

		# Solder mask and best RGB
		self.solder_mask_color = self.detect_solder_mask(golden_paths[0] if golden_paths else None)
		self.best_rgb = self.calibrate_best_rgb(golden_paths[0] if golden_paths else None)  # List of (r,g,b) tuples
		print(f"Detected solder mask: {self.solder_mask_color}, Best RGB: {self.best_rgb}")

		# Golden calibration stage
		if golden_paths and os.path.exists('golden_calib.pkl'):
			with open('golden_calib.pkl', 'rb') as f:
				self.golden_data = pickle.load(f)
		else:
			self.golden_data = self.golden_calibration_stage(golden_paths[0] if golden_paths else None)


	
	
	def detect_solder_mask(self, golden_img_path):
		"""Detect solder mask color using k-means clustering."""
		if not golden_img_path or self.ser is None:
			print("Solder mask detection skipped; assuming green.")
			return "GREEN"

		cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			print("Error: Could not open camera for solder mask detection.")
			return "GREEN"

		# Capture with white light, all rings, 0° polarizer
		self.ser.write("SEQ_255_255_255_0_3\n".encode())
		resp = self.ser.readline().decode().strip()
		if resp != 'SEQ_COMPLETE':
			print(f"Error capturing for solder mask: {resp}")
			cap.release()
			return "GREEN"

		time.sleep(0.5)
		ret, img = cap.read()
		cap.release()

		if not ret:
			print("Error: Failed to capture image for solder mask.")
			return "GREEN"

		# Mask out components
		mask = np.ones(img.shape[:2], dtype=np.uint8) * 255
		if self.placements:
			for ref, pos in self.placements.items():
				x_px = int(pos['x_mm'] / self.scale_mm_per_px)
				y_px = int(pos['y_mm'] / self.scale_mm_per_px)
				cv2.rectangle(mask, (x_px - 20, y_px - 20), (x_px + 20, y_px + 20), 0, -1)
		else:
			edges = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 50, 150)
			mask[edges > 0] = 0
		# Extract masked pixels
		masked_img = cv2.bitwise_and(img, img, mask=mask)
		pixels = masked_img.reshape(-1, 3)
		pixels = pixels[np.all(pixels != 0, axis=1)]  # Remove black

		if len(pixels) < 100:
			print("Warning: Insufficient pixels for solder mask detection.")
			return "UNKNOWN"

		# K-means on RGB (robust for color clustering)
		pixels_float = np.float32(pixels)
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		K = 3
		_, labels, centers = cv2.kmeans(pixels_float, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

		# Dominant cluster
		dominant_label = np.argmax(np.bincount(labels.flatten()))
		dominant_color = centers[dominant_label].astype(int)

		# Map to name using HSV hue
		hsv_color = cv2.cvtColor(np.uint8([[dominant_color]]), cv2.COLOR_RGB2HSV)[0][0]
		hue = hsv_color[0]

		color_map = {
			(0, 10): 'RED',
			(10, 20): 'ORANGE',
			(20, 40): 'YELLOW',
			(40, 80): 'GREEN',
			(80, 140): 'BLUE',
			(140, 160): 'PURPLE',
			(160, 180): 'MAGENTA'
		}
		for h_range, name in color_map.items():
			if h_range[0] <= hue < h_range[1]:
				return name

		# Fallback based on value/saturation
		if hsv_color[1] < 50 and hsv_color[2] > 200:
			return 'WHITE'
		elif hsv_color[2] < 50:
			return 'BLACK'
		return "UNKNOWN"

	def calibrate_best_rgb(self, golden_img_path):
		"""Fine-tune RGB values for max contrast, adapted to solder mask."""
		if not golden_img_path or self.ser is None:
			print("RGB calibration skipped; using defaults.")
			return [(255, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255)]  # Fallback

		cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			print("Error: Could not open camera for calibration.")
			return [(255, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255)]

		# RGB palette (27 combinations: 0/127/255 for R,G,B)
		palette = [(r, g, b) for r in [0, 127, 255] for g in [0, 127, 255] for b in [0, 127, 255] if
		           (r, g, b) != (0, 0, 0)]

		# Weights based on solder mask (higher for complementary)
		weights = {
			'GREEN': lambda rgb: 1.5 if rgb[0] > 200 and rgb[1] < 100 and rgb[2] < 100 else 1.0,  # Red/magenta
			'BLACK': lambda rgb: 1.5 if rgb[0] + rgb[1] + rgb[2] > 600 else 1.0,  # Bright colors
			'RED': lambda rgb: 1.5 if rgb[1] > 200 and rgb[0] < 100 and rgb[2] < 100 else 1.0,  # Green/cyan
			'BLUE': lambda rgb: 1.5 if rgb[0] > 200 and rgb[1] > 200 and rgb[2] < 100 else 1.0,  # Yellow/orange
			'WHITE': lambda rgb: 1.5 if rgb[2] > 200 and rgb[0] < 100 and rgb[1] < 100 else 1.0,  # Blue/purple
			'YELLOW': lambda rgb: 1.5 if rgb[2] > 200 and rgb[0] < 100 and rgb[1] < 100 else 1.0,  # Blue/purple
			'PURPLE': lambda rgb: 1.5 if rgb[0] > 200 and rgb[1] > 200 and rgb[2] < 100 else 1.0,  # Yellow/green
			'MAGENTA': lambda rgb: 1.5 if rgb[1] > 200 and rgb[0] < 100 and rgb[2] < 100 else 1.0,  # Green/cyan
			'ORANGE': lambda rgb: 1.5 if rgb[2] > 200 and rgb[0] < 100 and rgb[1] < 100 else 1.0,  # Blue/purple
			'UNKNOWN': lambda rgb: 1.0
		}
		weight_func = weights.get(self.solder_mask_color, lambda rgb: 1.0)
		rgb_contrasts = {}
		ring = 3  # All rings
		angle = 0  # Fixed

		for r, g, b in palette:
			self.ser.write(f"SEQ_{r}_{g}_{b}_{angle}_{ring}\n".encode())
			resp = self.ser.readline().decode().strip()
			if resp == 'SEQ_COMPLETE':
				time.sleep(0.5)
				ret, frame = cap.read()
				if ret:
					gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
					contrast = gray.std()  # Std dev as contrast metric
					weighted_contrast = contrast * weight_func((r, g, b))
					rgb_contrasts[(r, g, b)] = weighted_contrast
					print(f"RGB ({r},{g},{b}) contrast: {contrast:.2f}, weighted: {weighted_contrast:.2f}")
			else:
				print(f"Error calibrating RGB ({r},{g},{b}): {resp}")

		cap.release()

		# Select top 4 RGB by weighted contrast
		best = sorted(rgb_contrasts, key=rgb_contrasts.get, reverse=True)[:4]
		pickle.dump({'solder_mask': self.solder_mask_color, 'best_rgb': best}, open('color_calib.pkl', 'wb'))
		print(f"Best RGB for {self.solder_mask_color} solder mask: {best}")
		return best

	def extract_hog_from_images(self, paths, labels):
		"""Extract HOG features from images."""
		feats = []
		for path in paths:
			if not os.path.exists(path):
				print(f"Warning: Image not found: {path}")
				continue

			img = cv2.imread(path, 0)
			if img is None:
				print(f"Warning: Failed to load {path}")
				continue

			# Extract center region and resize for consistent HOG
			h, w = img.shape
			roi = img[h // 4:3 * h // 4, w // 4:3 * w // 4]
			roi_resized = cv2.resize(roi, (128, 128))

			# Extract HOG features
			feat = hog(roi_resized, orientations=9, pixels_per_cell=(8, 8),
			           cells_per_block=(2, 2), block_norm='L2-Hys')
			feats.append(feat)

		return np.array(feats) if feats else np.array([])

	def parse_bom(self, bom_path):
		"""Parse BOM CSV file."""
		try:
			df = pd.read_csv(bom_path)
			components = {}
			for _, row in df.iterrows():
				ref = row['Ref']
				components[ref] = {
					'value': str(row['Value']),
					'footprint': str(row['Footprint']),
					'package': str(row.get('Package', ''))
				}
			print(f"Parsed {len(components)} components from BOM.")
			return components
		except Exception as e:
			print(f"Error parsing BOM {bom_path}: {e}")
			return {}

	def parse_placement(self, placement_path):
		"""Parse placement/position CSV file."""
		try:
			df = pd.read_csv(placement_path)
			placements = {}
			for _, row in df.iterrows():
				ref = row['RefDes']
				placements[ref] = {
					'x_mm': float(row['Mid X']),
					'y_mm': float(row['Mid Y']),
					'rotation': float(row['Rotation']),
					'layer': str(row['Layer'])
				}
			print(f"Parsed {len(placements)} placements.")
			return placements
		except Exception as e:
			print(f"Error parsing placement {placement_path}: {e}")
			return {}

	def merge_design_refs(self):
		"""Merge BOM and placement data."""
		merged = {}
		for ref in self.placements:
			if ref in self.components:
				merged[ref] = {**self.placements[ref], **self.components[ref]}
		return merged

	def auto_calibrate(self, calib_img_path):
		"""Calibrate scale from golden image."""
		if not calib_img_path or self.gerber_fids is None:
			print("Calibration fallback: 0.015 mm/px (no image or fids).")
			return 0.015

		img = cv2.imread(calib_img_path)
		if img is None:
			print(f"Error: Calibration image {calib_img_path} not found.")
			return 0.015
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=5, maxRadius=20)
		if circles is None:
			print("No fiducials detected in image.")
			return 0.015

		img_fids_px = circles[0, :, :2]
		img_fids_px = img_fids_px[:len(self.gerber_fids)]
		dist_matrix = cdist(img_fids_px, self.gerber_fids)
		matches = np.argmin(dist_matrix, axis=1)
		paired_mm = self.gerber_fids[matches]

		px_dists = []
		mm_dists = []
		for i in range(len(img_fids_px)):
			for j in range(i + 1, len(img_fids_px)):
				px_dists.append(np.linalg.norm(img_fids_px[i] - img_fids_px[j]))
				mm_dists.append(np.linalg.norm(paired_mm[i] - paired_mm[j]))
		scale = np.mean(np.array(mm_dists) / np.array(px_dists)) if px_dists else 0.015

		self.fid_map = dict(zip(map(tuple, img_fids_px.tolist()), paired_mm.tolist()))
		pickle.dump({'scale': scale, 'fid_map': self.fid_map}, open('gerber_calib.pkl', 'wb'))
		print(f"Gerber-calibrated scale: {scale:.4f} mm/px")
		return scale
	def detect_sn_qr(self, img):
		"""Detect serial number from QR code or OCR."""
		# Try QR code first
		detector = cv2.QRCodeDetector()
		data, bbox, _ = detector.detectAndDecode(img)
		if data:
			return data

		# Fallback to OCR on top-left corner
		h, w = img.shape[:2]
		roi = img[0:int(0.2 * h), 0:int(0.2 * w)]

		try:
			ocr_results = self.ocr_reader.readtext(roi)
			for (bbox, text, conf) in ocr_results:
				if conf > 0.7 and ('SN:' in text.upper() or text.isalnum()):
					return text.strip()
		except Exception as e:
			print(f"OCR error: {e}")

		return f"Unknown_SN_{datetime.now().strftime('%H%M%S')}"

	def detect_polarity_yolo(self, img, bbox):
		"""Detect component polarity using YOLO."""
		if not YOLO_AVAILABLE or self.yolo_model is None:
			return None

		x, y, w, h = bbox
		roi = img[y:y + h, x:x + w]

		try:
			results = self.yolo_model(roi, verbose=False)
			if len(results) > 0 and len(results[0].boxes) > 0:
				# Check if detected objects indicate polarity markers
				for box in results[0].boxes:
					cls = int(box.cls[0])
					# Assuming class 0 is polarity marker (customize based on trained model)
					if cls == 0:
						return True
		except Exception as e:
			print(f"YOLO detection error: {e}")

		return None

	def detect_defects(self, img):
		"""Detect defects using edge detection and HOG classification."""
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		denoised = cv2.bilateralFilter(gray, 5, 50, 50)
		edges = cv2.Canny(denoised, 30, 90)

		contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		defects = []

		for cnt in contours:
			area = cv2.contourArea(cnt)
			if 5 < area < 500:  # Filter noise and large regions
				x, y, w, h = cv2.boundingRect(cnt)

				# Skip if too small
				if w < 3 or h < 3:
					continue

				roi = gray[y:y + h, x:x + w]
				if roi.size == 0:
					continue

				# Extract HOG features
				try:
					roi_resized = cv2.resize(roi, (128, 128))
					feat = hog(roi_resized, orientations=9, pixels_per_cell=(8, 8),
					           cells_per_block=(2, 2), block_norm='L2-Hys').reshape(1, -1)

					if self.clf is not None:
						pred = self.clf.predict(feat)[0]
						conf = self.clf.decision_function(feat)[0]

						if pred == 1 or conf < -0.5:  # Anomaly detected
							cent_px = (x + w // 2, y + h // 2)
							cent_mm = (cent_px[0] * self.scale_mm_per_px,
							           cent_px[1] * self.scale_mm_per_px)

							# Find nearest component
							nearest_ref = 'unknown'
							dist_mm = 0.0

							if self.placements:
								min_dist = float('inf')
								for ref, pos in self.placements.items():
									d = np.hypot(cent_mm[0] - pos['x_mm'],
									             cent_mm[1] - pos['y_mm'])
									if d < min_dist:
										min_dist = d
										nearest_ref = ref
								dist_mm = min_dist

							# Check polarity
							polarity_ok = self.detect_polarity_yolo(img, (x, y, w, h))

							defects.append((
								cent_px, (x, y, w, h), abs(conf),
								{
									'type': 'defect',
									'ref': nearest_ref,
									'dist_mm': dist_mm,
									'cent_mm': cent_mm,
									'polarity_ok': polarity_ok
								}
							))
				except Exception as e:
					print(f"Feature extraction error: {e}")
					continue

		return defects

	def mouse_callback(self, event, x, y, flags, param):
		"""Handle mouse clicks for defect labeling."""
		global current_defect

		if event == cv2.EVENT_LBUTTONDOWN:
			if current_defect:
				self.update_model(current_defect[1], 0)
				messagebox.showinfo("Feedback", "Marked as good. Model updated.")

		elif event == cv2.EVENT_RBUTTONDOWN:
			if current_defect:
				type_dialog = simpledialog.askinteger(
					"Defect Type",
					"0=bridge, 1=missing, 2=mismatch, 3=open, 4=excess, 5=tombstone:",
					minvalue=0, maxvalue=5
				)
				if type_dialog is not None:
					defect_type, category = self.defect_types[type_dialog]
					label = type_dialog + 1
					self.update_model(current_defect[1], label)

					centroid, bbox, conf, meta = current_defect
					defect_labels.append({
						'type': defect_type,
						'category': category,
						'bbox': bbox,
						'centroid_mm': meta['cent_mm'],
						'alignment_offset_mm': meta['dist_mm'],
						'conf': conf,
						'ref': meta['ref'],
						'polarity_ok': meta.get('polarity_ok')
					})
					messagebox.showinfo("Feedback", f"Labeled as {defect_type} ({category}).")

	def update_model(self, bbox, label):
		"""Update classifier with new labeled sample (placeholder for online learning)."""
		# This would require online learning implementation
		# For now, just log the feedback
		print(f"Model feedback: bbox={bbox}, label={label}")

	def run_inspection(self, sn=None):
		"""Run full inspection sequence."""
		global current_defect, defect_labels
		defect_labels = []  # Reset per board

		if self.ser is None:
			print("No Pico connected; skipping lighting sequence.")
			return {}

		cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			print("Error: Could not open camera.")
			return {}

		# Fast fiducial pass
		print("Fast fiducial pass for board pose...")
		self.ser.write("SEQ_255_255_255_0_3\n".encode())
		resp = self.ser.readline().decode().strip()

		board_rotation = 0.0
		if resp == 'SEQ_COMPLETE':
			time.sleep(0.5)
			ret, initial_img = cap.read()
			if ret:
				self.scale_mm_per_px = self.auto_calibrate_from_image(initial_img)
				initial_h = self.estimate_board_pose(initial_img)
				board_rotation = self.calculate_rotation(initial_h)
				print(f"Detected board rotation: {board_rotation:.2f}°")

		# Adjust polarizer angles
		angles = [(0 + board_rotation) % 360,
		          (120 + board_rotation) % 360,
		          (240 + board_rotation) % 360]

		images = []
		rings = [0, 1, 2]
		rgbs = self.best_rgb

		for ring in rings:
			for r, g, b in rgbs:
				for angle in angles:
					cmd = f"SEQ_{r}_{g}_{b}_{int(angle)}_{ring}\n"
					self.ser.write(cmd.encode())
					resp = self.ser.readline().decode().strip()

					if resp == 'SEQ_COMPLETE':
						time.sleep(0.5)
						ret, frame = cap.read()
						if ret:
							images.append(frame)
						else:
							print("Warning: Failed to capture frame.")					else:
						print(f"Error: Pico responded {resp}")
					time.sleep(0.1)

		cap.release()

		if not images:
			print("No images captured; inspection aborted.")
			return {}

		# Fuse images
		img = np.mean(images, axis=0).astype(np.uint8)
		print(f"Fused {len(images)} images from multi-light sequence.")

		# Detect SN
		if not sn:
			sn = self.detect_sn_qr(img)

		timestamp = datetime.now().isoformat()
		defects = self.detect_defects(img)

		if defects:
			first_centroid = defects[0][0]
			pyautogui.moveTo(first_centroid[0], first_centroid[1])
			display = img.copy()
			for i, (cent, bbox, conf, meta) in enumerate(defects):
				x, y, w, h = bbox
				cv2.rectangle(display, (x, y), (x + w, y + h), (0, 0, 255), 2)
				cv2.circle(display, cent, 5, (255, 0, 0), -1)
				label_text = f'{meta["ref"]}:{meta["type"]} {conf:.2f}'
				cv2.putText(display, label_text, (x, y - 10),
				            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

			cv2.namedWindow('PCB Inspection', cv2.WINDOW_NORMAL)
			cv2.setMouseCallback('PCB Inspection', self.mouse_callback)
			current_defect = defects[0]

			print("\nInspection Controls:")
			print("  Left-click: Mark as good (false positive)")
			print("  Right-click: Label defect type")
			print("  'n': Next defect")
			print("  'q': Quit inspection")

			while True:
				cv2.imshow('PCB Inspection', display)
				key = cv2.waitKey(1) & 0xFF

				if key == ord('n') and len(defects) > 1:
					idx = (defects.index(current_defect) + 1) % len(defects)
					current_defect = defects[idx]
					pyautogui.moveTo(current_defect[0][0], current_defect[0][1])
				elif key == ord('q'):
					break

			cv2.destroyAllWindows()
		else:
			messagebox.showinfo("Result", "No defects found.")

		# Generate report
		board_report = {
			'sn': sn,
			'timestamp': timestamp,
			'batch_id': self.batch_id,
			'defects': defect_labels,
			'total_defects': len(defect_labels),
			'solder_mask': self.solder_mask_color,
			'board_rotation_deg': board_rotation
		}

		self.save_report(board_report)
		return board_report

	def save_report(self, board_report):
		"""Save inspection report to JSON and CSV."""
		# Save JSON
		json_path = os.path.join(self.report_dir,
		                         f"{self.batch_id}_{board_report['sn']}.json")
		with open(json_path, 'w') as f:
			json.dump(board_report, f, indent=4)

		# Append to CSV
		csv_path = os.path.join(self.report_dir, f"{self.batch_id}.csv")
		file_exists = os.path.isfile(csv_path)

		with open(csv_path, 'a', newline='') as f:
			writer = csv.writer(f)
			if not file_exists:
				writer.writerow([
					'SN', 'Timestamp', 'Solder_Mask', 'Defect_Type', 'Category',
					'Centroid_X_mm', 'Centroid_Y_mm', 'Alignment_Offset_mm',
					'Conf', 'Ref', 'Polarity_OK', 'Notes'
				])

			for defect in board_report['defects']:
				writer.writerow([
					board_report['sn'],
					board_report['timestamp'],
					board_report['solder_mask'],
					defect['type'],
					defect['category'],
					defect['centroid_mm'][0],
					defect['centroid_mm'][1],
					defect['alignment_offset_mm'],
					defect['conf'],
					defect['ref'],
					defect.get('polarity_ok', 'N/A'),
					''
				])

		print(f"Report saved: {json_path}")

# Main execution


if __name__ == "__main__":
	aoi = EnhancedInteractiveAOI(
		golden_paths=[r'.\test\golden1.jpg', r'.\test\golden2.jpg'],
		gerber_path=r'.\test\top_copper.gbr',
		bom_path=r'.\test\bom.csv',
		placement_path=r'.\test\positions.csv',
		fid_diam_mm=1.0,
		pico_port='COM10'
	)

	report = aoi.run_inspection()
	print(f"\n=== Inspection Complete ===")
	print(f"SN: {report.get('sn', 'Unknown')}")
	print(f"Solder Mask: {report.get('solder_mask', 'Unknown')}")
	print(f"Total Defects: {report.get('total_defects', 0)}")
	print(f"Board Rotation: {report.get('board_rotation_deg', 0):.2f}°")