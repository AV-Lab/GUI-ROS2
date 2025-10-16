#!/usr/bin/env python3
import os
import shlex
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

# ---- Configuration ---------------------------------------------------------
# If you prefer the app to source ROS automatically, set ROS_SETUP to your distro’s setup:
# Example: ROS_SETUP = "/opt/ros/humble/setup.bash"
# If left as None, the app assumes you launched it from a terminal where ROS is already sourced.
ROS_SETUP = None  # or "/opt/ros/humble/setup.bash"

# ---------------------------------------------------------------------------

def run_shell_command(cmd: str) -> tuple[int, str, str]:
    """
    Run a shell command with /bin/bash -lc so we can 'source' things if needed.
    Returns (returncode, stdout, stderr).
    """
    # Build a command string that optionally sources ROS, then runs the cmd
    if ROS_SETUP:
        compound = f"source {shlex.quote(ROS_SETUP)} && {cmd}"
    else:
        compound = cmd

    proc = subprocess.run(
        ["/bin/bash", "-lc", compound],
        capture_output=True,
        text=True,
        env=os.environ.copy(),
    )
    return proc.returncode, proc.stdout, proc.stderr


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 CLI Launcher (Tkinter)")
        self.root.geometry("800x500")

        top = ttk.Frame(root, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        self.btn = ttk.Button(top, text="Show ROS 2 Topics", command=self.on_show_topics)
        self.btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)

        # Output area
        mid = ttk.Frame(root, padding=(10, 0))
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.txt = tk.Text(mid, wrap="none", height=24)
        self.txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Scrollbars
        yscroll = ttk.Scrollbar(mid, orient="vertical", command=self.txt.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt.configure(yscrollcommand=yscroll.set)

        xscroll = ttk.Scrollbar(root, orient="horizontal", command=self.txt.xview)
        xscroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.txt.configure(xscrollcommand=xscroll.set)

        # Hint if ROS may not be sourced
        if not os.environ.get("ROS_DISTRO") and not ROS_SETUP:
            self._append(
                "[hint] ROS_DISTRO is not set. Make sure you launched this from a terminal where ROS 2 is sourced,\n"
                "or set ROS_SETUP inside this script.\n\n"
            )

    def _append(self, text: str):
        self.txt.insert(tk.END, text)
        self.txt.see(tk.END)

    def on_show_topics(self):
        # Disable button while running
        self.btn.config(state=tk.DISABLED)
        self.status_var.set("Running: ros2 topic list ...")

        def worker():
            rc, out, err = run_shell_command("ros2 topic list")
            # Post results back to UI thread
            def done():
                self._append("$ ros2 topic list\n")
                if out.strip():
                    self._append(out)
                    if not out.endswith("\n"):
                        self._append("\n")
                if err.strip():
                    self._append("[stderr]\n" + err + ("\n" if not err.endswith("\n") else ""))

                if rc == 0:
                    self.status_var.set("Done")
                else:
                    self.status_var.set(f"Command failed (exit {rc})")
                    messagebox.showerror("ros2 topic list failed",
                                         f"Exit code: {rc}\n\nStderr:\n{err if err.strip() else '(empty)'}")
                self._append("\n")
                self.btn.config(state=tk.NORMAL)

            self.root.after(0, done)

        threading.Thread(target=worker, daemon=True).start()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
