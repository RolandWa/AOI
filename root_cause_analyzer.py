# Root Cause Analyzer Script
# Version: 1.0.0
# Date: 2025-10-21
# Author: Grok (xAI)
# Changelog:
#   - 1.0.0 (2025-10-21): Initial release with defect trend analysis, root cause suggestions
# Dependencies:
#   - Python==3.8.x
#   - pandas==1.3.5, matplotlib==3.5.1
# Notes: Run post-batch with `python root_cause_analyzer.py --batch_ids batch_20251021_1027`

import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime
import sys

# Version check
if sys.version_info < (3, 8):
	raise RuntimeError("Python 3.8+ required.")
if pd.__version__ < '1.3':
	print(f"Warning: pandas {pd.__version__} detected; 1.3.5 recommended.")

ROOT_CAUSES = {
	'paste_print': {
		'bridge': 'Excess paste volume - Tune stencil thickness or pressure.',
		'excess': 'Over-aperture in stencil - Verify CAD vs. stencil fab.',
		'missing': 'Clogged apertures - Clean stencil more frequently.'
	},
	'placement': {
		'missing': 'Mounter feeder jam - Check vacuum nozzles for 0402.',
		'mismatch': 'Vision misalignment - Calibrate pick-and-place offsets.',
		'tombstone': 'Nozzle height error - Adjust Z-axis for 0.5mm pitch ICs.'
	},
	'reflow': {
		'open': 'Insufficient wetting - Increase preheat ramp to 150-180°C.',
		'tombstone': 'Thermal gradient - Optimize conveyor speed to 1m/min.',
		'bridge': 'Solder balling - Reduce peak temp or add N2 atmosphere.'
	}
}


def analyze_reports(report_dir='reports', batch_ids=None):
	if not batch_ids:
		batch_ids = [f.split('.')[0] for f in os.listdir(report_dir) if f.endswith('.csv')]

	all_data = []
	for bid in batch_ids:
		csv_path = os.path.join(report_dir, f"{bid}.csv")
		if os.path.exists(csv_path):
			df = pd.read_csv(csv_path)
			df['batch_id'] = bid
			all_data.append(df)

	if not all_data:
		print("No reports found.")
		return

	df = pd.concat(all_data, ignore_index=True)

	summary = df.groupby(['Category', 'Defect_Type', 'Ref']).agg({
		'SN': 'count',
		'batch_id': 'nunique'
	}).rename(columns={'SN': 'Occurrences', 'batch_id': 'Affected_Batches'}).reset_index()

	summary['Root_Cause_Suggestion'] = summary.apply(
		lambda row: ROOT_CAUSES.get(row['Category'], {}).get(row['Defect_Type'], 'Investigate manually'), axis=1
	)

	summary_path = os.path.join(report_dir, f"analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
	summary.to_csv(summary_path, index=False)
	print(f"Summary saved: {summary_path}")

	fig, ax = plt.subplots(1, 2, figsize=(12, 5))
	df['Category'].value_counts().plot(kind='bar', ax=ax[0])
	ax[0].set_title('Defects by Process Stage')
	df['Defect_Type'].value_counts().plot(kind='bar', ax=ax[1])
	ax[1].set_title('Defect Types Distribution')
	plt.tight_layout()
	plt.savefig(os.path.join(report_dir, f"defect_trends_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"))
	plt.show()

	print("\nTop Root Causes:")
	for _, row in summary.nlargest(5, 'Occurrences').iterrows():
		print(
			f"- {row['Defect_Type']} ({row['Category']} @ {row['Ref']}): {row['Occurrences']} occ. | {row['Root_Cause_Suggestion']}")


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--batch_ids', nargs='+', help='Specific batch IDs to analyze')
	args = parser.parse_args()
	analyze_reports(batch_ids=args.batch_ids)