from Setup_AVL_Cam import Camera

avlcam = Camera("/dev/video0")
avlcam.cam_get_controls()
avlcam.cam_set_controls()
