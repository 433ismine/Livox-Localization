#!/usr/bin/env python3

import os

import numpy as np

from scan_context_manager import ScanContextManager

if __name__ == "__main__":
    date_path = "/home/dynamicx/rm_ws/src/rm_sentry/rm_navigation/global_localization"
    Livox_SC = ScanContextManager(file_path=date_path)

    pcd_path = "/home/dynamicx/rm_ws/src/rm_sentry/rm_navigation/global_path"
    sc_path = os.path.join(date_path, "scancontext")
    rk_path = os.path.join(date_path, "ringkey")
    npy_path = os.path.join(date_path, "pcd_npy")

    make_sc = True
    if make_sc:
        # Test the ScanContext Maker
        Livox_SC.livox_load_pc_make_sc(pcd_path)

    else:
        # Test the ScanContext Load and Localization
        Livox_SC.livox_load_sc_rk(sc_path, rk_path)
        for i in range(0, 300, 50):
            file_name = "pc_" + str(i) + ".npy"
            print("load test pc: ", file_name)
            test_pc = os.path.join(npy_path, file_name)
            test_trans = Livox_SC.initialization(np.load(test_pc))
            print(test_trans)
