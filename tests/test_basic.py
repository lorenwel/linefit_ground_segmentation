from linefit import ground_seg

def tests_groundseg():
    mydufo = ground_seg() # resolution, d_s, d_p, hit_extension
    print("Hello Ground Segmentation!")
    mydufo.clean()
    print("Cleaned Ground Segmentation!")

if __name__ == "__main__":
    tests_groundseg()