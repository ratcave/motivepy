__author__ = 'ratcave'

 import motive as m

 m.load_project("video8.ttp")

 m.update()

 m.update()

 cams=m.get_cams()

pix=cams[6].pixel_resolution()

cams[6].get_frame_buffer(pix["width"],pix['height'])

Width_in_Bytes=Width_in_Pixels
Bits_per_Pixel=8

cdef unsigned char * buffer=<unsigned char *> malloc(Width_in_Bytes*Height_in_Pixels*sizeof(unsigned char))
        if TT_CameraFrameBuffer(self.index, Width_in_Pixels, Height_in_Pixels, Width_in_Bytes, Bits_per_Pixel, buffer):
                        #python automatically converts unsigned char to integer
            a=np.zeros((Width_in_Bytes,Height_in_Pixels), dtype='B')
            h=0
            while h-1<Height_in_Pixels:
                for w in xrange(Width_in_Bytes):
                    a[h][w]=buffer[h*Width_in_Bytes+w]
                h=h+1
                import pdb
                pdb.set_trace()