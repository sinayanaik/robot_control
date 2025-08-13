#!/usr/bin/env python3
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import pandas as pd
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


class SubplotWindow:
    def __init__(self, parent: tk.Tk, title: str) -> None:
        self.win = tk.Toplevel(parent)
        self.win.title(title)
        self.fig = Figure(figsize=(8, 6), constrained_layout=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.win)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.win, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.grid(row=1, column=0, sticky='ew')
        self.win.rowconfigure(0, weight=1)
        self.win.columnconfigure(0, weight=1)

    def render_joints(self, df: pd.DataFrame, t: pd.Series, joints: list[str], show_des: bool, show_act: bool) -> None:
        self.fig.clear()
        axes = self.fig.subplots(3, 1, sharex=True)
        for i in range(3):
            ax = axes[i]
            if i < len(joints) and joints[i]:
                j = joints[i]
                if show_des and ('des_' + j) in df.columns:
                    ax.plot(t, df['des_' + j], label='des', linewidth=1.2, alpha=0.9)
                if show_act and ('act_' + j) in df.columns:
                    ax.plot(t, df['act_' + j], label='act', linewidth=1.2, alpha=0.9)
                ax.set_ylabel(j)
                ax.legend(loc='upper right', fontsize='small')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('t [s]')
        self.canvas.draw_idle()

    def render_ee(self, df: pd.DataFrame, t: pd.Series, show_des: bool, show_act: bool) -> None:
        self.fig.clear()
        axes = self.fig.subplots(3, 1, sharex=True)
        pairs = [('x', 'ee_des_x', 'ee_act_x'), ('y', 'ee_des_y', 'ee_act_y'), ('z', 'ee_des_z', 'ee_act_z')]
        for i, (lab, dcol, acol) in enumerate(pairs):
            ax = axes[i]
            if show_des and dcol in df.columns:
                ax.plot(t, df[dcol], label='des', linewidth=1.2, alpha=0.9)
            if show_act and acol in df.columns:
                ax.plot(t, df[acol], label='act', linewidth=1.2, alpha=0.9)
            ax.set_ylabel('ee_' + lab)
            ax.legend(loc='upper right', fontsize='small')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('t [s]')
        self.canvas.draw_idle()


class LogViewer:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Arm Log Viewer')

        self.df: pd.DataFrame | None = None
        self.file_paths: list[Path] = []
        self.joint_names: list[str] = []

        self.show_desired = tk.BooleanVar(value=True)
        self.show_actual = tk.BooleanVar(value=True)
        self.show_ee_des = tk.BooleanVar(value=True)
        self.show_ee_act = tk.BooleanVar(value=True)
        self.max_points_var = tk.IntVar(value=2000)

        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky='nsew')
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky='ns')
        # spacer to keep layout simple
        right = ttk.Frame(main, width=1)
        right.grid(row=0, column=1, sticky='ns', padx=(8, 0))
        main.columnconfigure(1, weight=0)
        main.rowconfigure(0, weight=1)

        ttk.Button(left, text='Open CSV...', command=self.on_open_csv).grid(row=0, column=0, sticky='ew')
        self.files_list = tk.Listbox(left, height=10)
        self.files_list.grid(row=1, column=0, sticky='ns', pady=(6, 0))
        self.files_list.bind('<<ListboxSelect>>', self.on_select_file)
        left.rowconfigure(1, weight=1)

        opts = ttk.LabelFrame(left, text='Options', padding=6)
        opts.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        ttk.Checkbutton(opts, text='Joints desired', variable=self.show_desired, command=self.render).grid(row=0, column=0, sticky='w')
        ttk.Checkbutton(opts, text='Joints actual', variable=self.show_actual, command=self.render).grid(row=1, column=0, sticky='w')
        ttk.Checkbutton(opts, text='EE desired', variable=self.show_ee_des, command=self.render).grid(row=2, column=0, sticky='w')
        ttk.Checkbutton(opts, text='EE actual', variable=self.show_ee_act, command=self.render).grid(row=3, column=0, sticky='w')
        ttk.Label(opts, text='Max points').grid(row=4, column=0, sticky='w', pady=(6, 0))
        ttk.Entry(opts, textvariable=self.max_points_var, width=10).grid(row=4, column=1, sticky='w', padx=(6, 0))
        ttk.Button(opts, text='Apply', command=self.render).grid(row=4, column=2, sticky='w', padx=(6, 0))

        # plotting windows (created lazily)
        self.win_j1: SubplotWindow | None = None
        self.win_j2: SubplotWindow | None = None
        self.win_ee: SubplotWindow | None = None

    def on_open_csv(self) -> None:
        paths = filedialog.askopenfilenames(title='Select CSV files', filetypes=[('CSV files', '*.csv')])
        if not paths:
            return
        self.file_paths = [Path(p) for p in paths]
        self.files_list.delete(0, tk.END)
        for p in self.file_paths:
            self.files_list.insert(tk.END, str(p))
        self.files_list.selection_clear(0, tk.END)
        self.files_list.selection_set(0)
        self.files_list.event_generate('<<ListboxSelect>>')

    def on_select_file(self, _evt=None) -> None:
        sel = self.files_list.curselection()
        if not sel:
            return
        idx = sel[0]
        try:
            self.load_csv(self.file_paths[idx])
            self.render()
        except Exception as e:
            messagebox.showerror('Error', f'Failed to load file: {e}')

    def load_csv(self, path: Path) -> None:
        df = pd.read_csv(path)
        if 't' not in df.columns:
            raise ValueError('CSV missing column t')
        self.df = df
        des_cols = [c for c in df.columns if c.startswith('des_') and not c.startswith('des_ee_')]
        names = [c[4:] for c in des_cols]
        self.joint_names = [n for n in names if ('act_' + n) in df.columns]

    def render(self) -> None:
        if self.df is None:
            return
        df = self.df
        max_points = max(100, int(self.max_points_var.get() or 2000))
        if len(df) > max_points:
            idx = np.linspace(0, len(df) - 1, max_points).astype(int)
            df = df.iloc[idx].reset_index(drop=True)
        t = df['t']

        g1 = self.joint_names[:3]
        g2 = self.joint_names[3:6]

        if self.win_j1 is None or not self._window_alive(self.win_j1.win):
            self.win_j1 = SubplotWindow(self.root, 'Joints 1-3')
        if self.win_j2 is None or not self._window_alive(self.win_j2.win):
            self.win_j2 = SubplotWindow(self.root, 'Joints 4-6')
        if self.win_ee is None or not self._window_alive(self.win_ee.win):
            self.win_ee = SubplotWindow(self.root, 'End Effector (x,y,z)')

        self.win_j1.render_joints(df, t, g1, self.show_desired.get(), self.show_actual.get())
        self.win_j2.render_joints(df, t, g2, self.show_desired.get(), self.show_actual.get())
        self.win_ee.render_ee(df, t, self.show_ee_des.get(), self.show_ee_act.get())

    @staticmethod
    def _window_alive(win: tk.Toplevel) -> bool:
        try:
            return bool(win.winfo_exists())
        except Exception:
            return False


def main() -> None:
    root = tk.Tk()
    app = LogViewer(root)
    if len(sys.argv) > 1:
        try:
            paths = []
            for arg in sys.argv[1:]:
                p = Path(arg)
                if p.is_dir():
                    paths += sorted(p.glob('*.csv'))
                elif p.is_file():
                    paths.append(p)
            if paths:
                app.file_paths = [Path(p) for p in paths]
                app.files_list.delete(0, tk.END)
                for p in app.file_paths:
                    app.files_list.insert(tk.END, str(p))
                app.files_list.selection_set(0)
                app.on_select_file()
        except Exception:
            pass
    root.mainloop()


if __name__ == '__main__':
    main()


