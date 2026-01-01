#!/usr/bin/env python
"""
Dataset Utilities for Robot Manipulation
- Verify collected data
- Visualize episodes
- Convert formats
- Compute statistics
"""

import zarr
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import cv2


class DatasetAnalyzer:
    """Analyze and visualize robot manipulation datasets"""
    
    def __init__(self, dataset_path):
        """
        Args:
            dataset_path: Path to .zarr dataset directory
        """
        self.dataset_path = Path(dataset_path)
        if not self.dataset_path.exists():
            raise FileNotFoundError(f"Dataset not found: {dataset_path}")
        
        # Open Zarr store
        self.root = zarr.open(str(dataset_path), mode='r')
        self.data = self.root['data']
        self.meta = self.root['meta']
        
        # Load data arrays
        self.images = self.data['img']
        self.actions = self.data['action']
        self.states = self.data['state']
        self.timestamps = self.data['timestamp']
        self.episode_ends = self.meta['episode_ends'][:]
        
        print(f"✅ Loaded dataset: {dataset_path}")
        self.print_summary()
    
    
    def print_summary(self):
        """Print dataset summary"""
        n_episodes = len(self.episode_ends)
        n_timesteps = self.images.shape[0]
        
        print("\n" + "="*60)
        print("📊 DATASET SUMMARY")
        print("="*60)
        print(f"Total episodes: {n_episodes}")
        print(f"Total timesteps: {n_timesteps}")
        print(f"Average episode length: {n_timesteps / n_episodes:.1f}")
        print(f"\nData shapes:")
        print(f"  Images:  {self.images.shape} (H, W, C)")
        print(f"  Actions: {self.actions.shape}")
        print(f"  States:  {self.states.shape}")
        print(f"\nEpisode lengths:")
        
        prev_end = 0
        for i, end in enumerate(self.episode_ends):
            length = end - prev_end
            print(f"  Episode {i+1}: {length} timesteps")
            prev_end = end
        
        print("="*60)
    
    
    def verify_data_integrity(self):
        """Check for data integrity issues"""
        print("\n🔍 Verifying data integrity...")
        
        issues = []
        
        # Check shape consistency
        n_imgs = self.images.shape[0]
        n_actions = self.actions.shape[0]
        n_states = self.states.shape[0]
        n_timestamps = self.timestamps.shape[0]
        
        if not (n_imgs == n_actions == n_states == n_timestamps):
            issues.append(f"Shape mismatch: imgs={n_imgs}, actions={n_actions}, states={n_states}, ts={n_timestamps}")
        
        # Check episode boundaries
        if len(self.episode_ends) > 0:
            if self.episode_ends[-1] != n_imgs:
                issues.append(f"Last episode end ({self.episode_ends[-1]}) doesn't match dataset size ({n_imgs})")
        
        # Check for NaN values
        if np.isnan(self.actions[:]).any():
            issues.append("NaN values found in actions")
        if np.isnan(self.states[:]).any():
            issues.append("NaN values found in states")
        
        # Check image value range
        img_min = self.images[:].min()
        img_max = self.images[:].max()
        if img_min < 0 or img_max > 1:
            issues.append(f"Image values out of [0,1] range: min={img_min:.3f}, max={img_max:.3f}")
        
        # Check action/state value ranges
        action_stats = {
            'min': self.actions[:].min(axis=0),
            'max': self.actions[:].max(axis=0)
        }
        state_stats = {
            'min': self.states[:].min(axis=0),
            'max': self.states[:].max(axis=0)
        }
        
        print(f"\n📐 Action ranges (per dimension):")
        for i, (mi, ma) in enumerate(zip(action_stats['min'], action_stats['max'])):
            print(f"  Dim {i}: [{mi:.1f}, {ma:.1f}]")
        
        print(f"\n📐 State ranges (per dimension):")
        for i, (mi, ma) in enumerate(zip(state_stats['min'], state_stats['max'])):
            print(f"  Dim {i}: [{mi:.1f}, {ma:.1f}]")
        
        # Report issues
        if issues:
            print(f"\n❌ Found {len(issues)} issues:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("\n✅ No integrity issues found!")
        
        return len(issues) == 0
    
    
    def get_episode(self, episode_idx):
        """
        Get data for a specific episode.
        
        Args:
            episode_idx: Episode index (0-based)
        
        Returns:
            dict with keys: images, actions, states, timestamps
        """
        if episode_idx < 0 or episode_idx >= len(self.episode_ends):
            raise ValueError(f"Episode {episode_idx} out of range [0, {len(self.episode_ends)-1}]")
        
        start = 0 if episode_idx == 0 else self.episode_ends[episode_idx - 1]
        end = self.episode_ends[episode_idx]
        
        return {
            'images': self.images[start:end],
            'actions': self.actions[start:end],
            'states': self.states[start:end],
            'timestamps': self.timestamps[start:end]
        }
    
    
    def visualize_episode(self, episode_idx, save_video=False, output_path="episode.mp4"):
        """
        Visualize an episode with image frames and action/state plots.
        
        Args:
            episode_idx: Episode to visualize
            save_video: If True, save as MP4
            output_path: Video output path
        """
        episode_data = self.get_episode(episode_idx)
        images = episode_data['images']
        actions = episode_data['actions']
        states = episode_data['states']
        
        n_frames = len(images)
        print(f"\n🎬 Visualizing Episode {episode_idx} ({n_frames} frames)")
        
        # Setup video writer if saving
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 10
            frame_size = (1200, 800)
            video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
        
        # Create figure
        fig = plt.figure(figsize=(12, 8))
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        ax_img = fig.add_subplot(gs[:2, 0])
        ax_action = fig.add_subplot(gs[0, 1])
        ax_state = fig.add_subplot(gs[1, 1])
        ax_diff = fig.add_subplot(gs[2, :])
        
        for frame_idx in range(n_frames):
            # Clear axes
            ax_img.clear()
            ax_action.clear()
            ax_state.clear()
            ax_diff.clear()
            
            # Plot image
            img = images[frame_idx]
            ax_img.imshow(img)
            ax_img.set_title(f"Frame {frame_idx}/{n_frames}")
            ax_img.axis('off')
            
            # Plot actions (up to current frame)
            for dim in range(actions.shape[1]):
                ax_action.plot(actions[:frame_idx+1, dim], label=f'Dim {dim}', alpha=0.7)
            ax_action.set_title("Actions")
            ax_action.set_xlabel("Timestep")
            ax_action.legend(fontsize='small', ncol=2)
            ax_action.grid(True, alpha=0.3)
            
            # Plot states
            for dim in range(states.shape[1]):
                ax_state.plot(states[:frame_idx+1, dim], label=f'Dim {dim}', alpha=0.7)
            ax_state.set_title("States")
            ax_state.set_xlabel("Timestep")
            ax_state.legend(fontsize='small', ncol=2)
            ax_state.grid(True, alpha=0.3)
            
            # Plot action-state difference
            diff = np.abs(actions[:frame_idx+1] - states[:frame_idx+1])
            ax_diff.plot(diff.mean(axis=1), color='red', linewidth=2)
            ax_diff.set_title("Action-State Tracking Error (L1)")
            ax_diff.set_xlabel("Timestep")
            ax_diff.set_ylabel("Mean Absolute Error")
            ax_diff.grid(True, alpha=0.3)
            
            plt.suptitle(f"Episode {episode_idx} - Frame {frame_idx}/{n_frames}", fontsize=16, fontweight='bold')
            
            # Save frame to video
            if save_video:
                fig.canvas.draw()
                frame = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame = cv2.resize(frame, frame_size)
                video_writer.write(frame)
            
            plt.pause(0.1)
        
        if save_video:
            video_writer.release()
            print(f"✅ Video saved to: {output_path}")
        
        plt.show()
    
    
    def plot_statistics(self):
        """Plot overall dataset statistics"""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Episode length histogram
        prev_end = 0
        episode_lengths = []
        for end in self.episode_ends:
            episode_lengths.append(end - prev_end)
            prev_end = end
        
        axes[0, 0].hist(episode_lengths, bins=20, edgecolor='black')
        axes[0, 0].set_title("Episode Length Distribution")
        axes[0, 0].set_xlabel("Length (timesteps)")
        axes[0, 0].set_ylabel("Frequency")
        axes[0, 0].grid(True, alpha=0.3)
        
        # Action distribution
        actions_flat = self.actions[:].reshape(-1)
        axes[0, 1].hist(actions_flat, bins=50, edgecolor='black', alpha=0.7)
        axes[0, 1].set_title("Action Value Distribution")
        axes[0, 1].set_xlabel("Value")
        axes[0, 1].set_ylabel("Frequency")
        axes[0, 1].grid(True, alpha=0.3)
        
        # State distribution
        states_flat = self.states[:].reshape(-1)
        axes[0, 2].hist(states_flat, bins=50, edgecolor='black', alpha=0.7, color='orange')
        axes[0, 2].set_title("State Value Distribution")
        axes[0, 2].set_xlabel("Value")
        axes[0, 2].set_ylabel("Frequency")
        axes[0, 2].grid(True, alpha=0.3)
        
        # Image brightness distribution
        img_mean_brightness = self.images[:].mean(axis=(1, 2, 3))
        axes[1, 0].hist(img_mean_brightness, bins=50, edgecolor='black', color='green')
        axes[1, 0].set_title("Image Mean Brightness")
        axes[1, 0].set_xlabel("Mean Pixel Value [0-1]")
        axes[1, 0].set_ylabel("Frequency")
        axes[1, 0].grid(True, alpha=0.3)
        
        # Action vs State correlation
        action_mean = self.actions[:].mean(axis=0)
        state_mean = self.states[:].mean(axis=0)
        dims = np.arange(len(action_mean))
        axes[1, 1].bar(dims - 0.2, action_mean, 0.4, label='Action', alpha=0.7)
        axes[1, 1].bar(dims + 0.2, state_mean, 0.4, label='State', alpha=0.7)
        axes[1, 1].set_title("Mean Action vs State (per dim)")
        axes[1, 1].set_xlabel("Dimension")
        axes[1, 1].set_ylabel("Mean Value")
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        # Timestamp intervals
        if len(self.timestamps[:]) > 1:
            dt = np.diff(self.timestamps[:])
            axes[1, 2].hist(dt, bins=50, edgecolor='black', color='purple')
            axes[1, 2].set_title("Timestamp Intervals")
            axes[1, 2].set_xlabel("dt (seconds)")
            axes[1, 2].set_ylabel("Frequency")
            axes[1, 2].axvline(dt.mean(), color='red', linestyle='--', label=f'Mean: {dt.mean():.3f}s')
            axes[1, 2].legend()
            axes[1, 2].grid(True, alpha=0.3)
        
        plt.suptitle("Dataset Statistics", fontsize=16, fontweight='bold')
        plt.tight_layout()
        plt.show()
    
    
    def export_to_numpy(self, output_path="dataset.npz"):
        """Export dataset to compressed numpy format"""
        print(f"\n💾 Exporting to {output_path}...")
        
        np.savez_compressed(
            output_path,
            images=self.images[:],
            actions=self.actions[:],
            states=self.states[:],
            timestamps=self.timestamps[:],
            episode_ends=self.episode_ends
        )
        
        print(f"✅ Exported to {output_path}")


# ==============================================================================
# CLI INTERFACE
# ==============================================================================
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Dataset Analysis Tool")
    parser.add_argument("dataset_path", help="Path to .zarr dataset")
    parser.add_argument("--verify", action="store_true", help="Verify data integrity")
    parser.add_argument("--stats", action="store_true", help="Show statistics plots")
    parser.add_argument("--visualize", type=int, metavar="EPISODE", help="Visualize episode")
    parser.add_argument("--export", type=str, metavar="PATH", help="Export to .npz")
    parser.add_argument("--save-video", action="store_true", help="Save visualization as video")
    
    args = parser.parse_args()
    
    # Load dataset
    analyzer = DatasetAnalyzer(args.dataset_path)
    
    # Run requested operations
    if args.verify:
        analyzer.verify_data_integrity()
    
    if args.stats:
        analyzer.plot_statistics()
    
    if args.visualize is not None:
        video_path = "episode.mp4" if args.save_video else None
        analyzer.visualize_episode(
            args.visualize, 
            save_video=args.save_video,
            output_path=video_path or "episode.mp4"
        )
    
    if args.export:
        analyzer.export_to_numpy(args.export)


if __name__ == "__main__":
    main()