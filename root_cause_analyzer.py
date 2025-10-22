# Root Cause Analyzer Script - FIXED
# Version: 2.0.1
# Date: 2025-10-22
# Critical Fixes:
#   - Added proper column validation and defaults
#   - Fixed aggregation to handle missing Alignment_Offset_mm
#   - Added polarity error analysis
#   - Improved error handling

import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime
import sys

# Version check
if sys.version_info < (3, 8):
	raise RuntimeError("Python 3.8+ required.")

ROOT_CAUSES = {
	'paste_print': {
		'bridge': 'Excess paste volume - Tune stencil thickness or reduce pressure.',
		'excess': 'Over-aperture in stencil - Verify CAD vs. stencil fabrication.',
		'missing': 'Clogged apertures - Clean stencil more frequently or check alignment.'
	},
	'placement': {
		'missing': 'Mounter feeder jam or empty - Check vacuum nozzles and feeders.',
		'mismatch': 'Vision misalignment or wrong reel - Calibrate pick-and-place vision.',
		'tombstone': 'Uneven nozzle height - Adjust Z-axis for component pitch.'
	},
	'reflow': {
		'open': 'Insufficient wetting - Increase preheat ramp to 150-180°C.',
		'tombstone': 'Thermal gradient across board - Optimize conveyor speed.',
		'bridge': 'Solder balling - Reduce peak temp by 5-10°C or add N2 atmosphere.'
	}
}

POLARITY_ROOT_CAUSE = 'Polarity mismatch - Check pick-and-place vision or feeder orientation (tape direction).'


def analyze_reports(report_dir='reports', batch_ids=None):
	"""Analyze defect reports and generate root cause insights."""

	if not os.path.exists(report_dir):
		print(f"Error: Report directory '{report_dir}' not found.")
		return

	if not batch_ids:
		# Auto-detect all batch CSV files
		batch_ids = []
		for f in os.listdir(report_dir):
			if f.endswith('.csv') and f.startswith('batch_'):
				batch_ids.append(f.replace('.csv', ''))

	if not batch_ids:
		print("No batch reports found.")
		return

	all_data = []
	for bid in batch_ids:
		csv_path = os.path.join(report_dir, f"{bid}.csv")
		if os.path.exists(csv_path):
			try:
				df = pd.read_csv(csv_path)
				df['batch_id'] = bid
				all_data.append(df)
				print(f"Loaded {len(df)} defects from {bid}")
			except Exception as e:
				print(f"Error reading {csv_path}: {e}")

	if not all_data:
		print("No valid reports found.")
		return

	df = pd.concat(all_data, ignore_index=True)
	print(f"\nTotal defects across all batches: {len(df)}")

	# Ensure required columns exist
	required_cols = ['Category', 'Defect_Type', 'Ref', 'SN']
	missing_cols = [col for col in required_cols if col not in df.columns]
	if missing_cols:
		print(f"Error: Missing required columns: {missing_cols}")
		return

	# Add Alignment_Offset_mm if missing (for backward compatibility)
	if 'Alignment_Offset_mm' not in df.columns:
		print("Warning: 'Alignment_Offset_mm' column not found. Adding with default values.")
		df['Alignment_Offset_mm'] = 0.0

	# Add Polarity_OK if missing
	if 'Polarity_OK' not in df.columns:
		df['Polarity_OK'] = 'N/A'

	# Replace NaN in Alignment_Offset_mm with 0
	df['Alignment_Offset_mm'] = df['Alignment_Offset_mm'].fillna(0.0)

	# Aggregate by defect type
	summary = df.groupby(['Category', 'Defect_Type', 'Ref']).agg({
		'SN': 'count',
		'batch_id': 'nunique',
		'Alignment_Offset_mm': 'mean'
	}).rename(columns={
		'SN': 'Occurrences',
		'batch_id': 'Affected_Batches',
		'Alignment_Offset_mm': 'Avg_Offset_mm'
	}).reset_index()

	# Add root cause suggestions
	summary['Root_Cause_Suggestion'] = summary.apply(
		lambda row: ROOT_CAUSES.get(row['Category'], {}).get(
			row['Defect_Type'], 'Manual investigation required'
		), axis=1
	)

	# Analyze polarity errors
	polarity_errors = df[df['Polarity_OK'] == False] if 'Polarity_OK' in df.columns else pd.DataFrame()
	if not polarity_errors.empty:
		print(f"\nFound {len(polarity_errors)} polarity errors")
		polarity_summary = polarity_errors.groupby('Ref').size().reset_index(name='Polarity_Errors')
		polarity_summary['Root_Cause'] = POLARITY_ROOT_CAUSE

		# Save polarity report
		polarity_path = os.path.join(report_dir, f"polarity_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
		polarity_summary.to_csv(polarity_path, index=False)
		print(f"Polarity analysis saved: {polarity_path}")

	# Save summary
	summary_path = os.path.join(report_dir, f"analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
	summary.to_csv(summary_path, index=False)
	print(f"\nSummary saved: {summary_path}")

	# Visualizations
	fig, axes = plt.subplots(2, 2, figsize=(15, 10))

	# Plot 1: Defects by Process Stage
	if 'Category' in df.columns:
		category_counts = df['Category'].value_counts()
		category_counts.plot(kind='bar', ax=axes[0, 0], color='steelblue')
		axes[0, 0].set_title('Defects by Process Stage', fontsize=14, fontweight='bold')
		axes[0, 0].set_xlabel('Process Stage')
		axes[0, 0].set_ylabel('Count')
		axes[0, 0].tick_params(axis='x', rotation=45)

	# Plot 2: Defect Types Distribution
	if 'Defect_Type' in df.columns:
		defect_counts = df['Defect_Type'].value_counts()
		defect_counts.plot(kind='bar', ax=axes[0, 1], color='coral')
		axes[0, 1].set_title('Defect Types Distribution', fontsize=14, fontweight='bold')
		axes[0, 1].set_xlabel('Defect Type')
		axes[0, 1].set_ylabel('Count')
		axes[0, 1].tick_params(axis='x', rotation=45)

	# Plot 3: Alignment Offsets Histogram
	if 'Alignment_Offset_mm' in df.columns:
		valid_offsets = df['Alignment_Offset_mm'].dropna()
		if len(valid_offsets) > 0:
			axes[1, 0].hist(valid_offsets, bins=20, color='lightgreen', edgecolor='black')
			axes[1, 0].set_title('Alignment Offsets Distribution', fontsize=14, fontweight='bold')
			axes[1, 0].set_xlabel('Offset (mm)')
			axes[1, 0].set_ylabel('Frequency')
			axes[1, 0].axvline(valid_offsets.mean(), color='red', linestyle='--',
			                   label=f'Mean: {valid_offsets.mean():.3f}mm')
			axes[1, 0].legend()

	# Plot 4: Top 10 Problematic Components
	if 'Ref' in df.columns:
		top_refs = df['Ref'].value_counts().head(10)
		top_refs.plot(kind='barh', ax=axes[1, 1], color='mediumpurple')
		axes[1, 1].set_title('Top 10 Problematic Components', fontsize=14, fontweight='bold')
		axes[1, 1].set_xlabel('Defect Count')
		axes[1, 1].set_ylabel('Component Reference')
		axes[1, 1].invert_yaxis()

	plt.tight_layout()
	plot_path = os.path.join(report_dir, f"defect_trends_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
	plt.savefig(plot_path, dpi=150, bbox_inches='tight')
	print(f"Trend plots saved: {plot_path}")
	plt.show()

	# Print top root causes
	print("\n" + "=" * 80)
	print("TOP ROOT CAUSES (by occurrence)")
	print("=" * 80)

	for i, row in summary.nlargest(10, 'Occurrences').iterrows():
		print(f"\n{i + 1}. {row['Defect_Type'].upper()} @ {row['Ref']} ({row['Category']})")
		print(f"   Occurrences: {row['Occurrences']}")
		print(f"   Affected Batches: {row['Affected_Batches']}")
		print(f"   Avg Alignment Offset: {row['Avg_Offset_mm']:.3f} mm")
		print(f"   → {row['Root_Cause_Suggestion']}")

	# Statistical summary
	print("\n" + "=" * 80)
	print("STATISTICAL SUMMARY")
	print("=" * 80)
	print(f"Total Boards Inspected: {df['SN'].nunique()}")
	print(f"Total Defects Found: {len(df)}")
	print(f"Defect Rate: {len(df) / df['SN'].nunique():.2f} defects/board")
	print(f"Most Common Defect: {df['Defect_Type'].mode()[0] if len(df) > 0 else 'N/A'}")
	print(f"Most Problematic Stage: {df['Category'].mode()[0] if len(df) > 0 else 'N/A'}")

	if 'Alignment_Offset_mm' in df.columns:
		valid_offsets = df['Alignment_Offset_mm'].dropna()
		if len(valid_offsets) > 0:
			print(f"Mean Alignment Offset: {valid_offsets.mean():.3f} mm")
			print(f"Max Alignment Offset: {valid_offsets.max():.3f} mm")

	if not polarity_errors.empty:
		print(f"Polarity Errors: {len(polarity_errors)} ({len(polarity_errors) / len(df) * 100:.1f}%)")


if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description='Analyze AOI defect reports and identify root causes'
	)
	parser.add_argument(
		'--batch_ids',
		nargs='+',
		help='Specific batch IDs to analyze (e.g., batch_20251022_1027)'
	)
	parser.add_argument(
		'--report_dir',
		default='reports',
		help='Directory containing report CSV files (default: reports)'
	)

	args = parser.parse_args()

	print("=" * 80)
	print("AOI ROOT CAUSE ANALYZER v2.0.1")
	print("=" * 80)

	analyze_reports(report_dir=args.report_dir, batch_ids=args.batch_ids)