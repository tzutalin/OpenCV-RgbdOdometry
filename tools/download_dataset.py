#!/usr/bin/env python
import os
import sys
import shutil
from subprocess import call
import tarfile
if sys.version_info >= (3,):
    import urllib.request as urllib2
    import urllib.parse as urlparse
else:
    import urllib2
    import urlparse


def untar(fname, dst='.'):
    if (fname.endswith("tar")):
        tar = tarfile.open(fname)
        tar.extractall(dst)
        tar.close()

def extractTarfile(filename):
    tar = tarfile.open(filename)
    tar.extractall()
    tar.close()

def download_file(url, desc=None, renamed_file=None):
    u = urllib2.urlopen(url)

    scheme, netloc, path, query, fragment = urlparse.urlsplit(url)
    filename = os.path.basename(path)
    if not filename:
        filename = 'downloaded.file'

    if not renamed_file is None:
        filename = renamed_file

    if desc:
        filename = os.path.join(desc, filename)

    with open(filename, 'wb') as f:
        meta = u.info()
        meta_func = meta.getheaders if hasattr(meta, 'getheaders') else meta.get_all
        meta_length = meta_func("Content-Length")
        file_size = None
        if meta_length:
            file_size = int(meta_length[0])
        print("Downloading: {0} Bytes: {1}".format(url, file_size))

        file_size_dl = 0
        block_sz = 8192
        while True:
            buffer = u.read(block_sz)
            if not buffer:
                break

            file_size_dl += len(buffer)
            f.write(buffer)

            status = "{0:16}".format(file_size_dl)
            if file_size:
                status += "   [{0:6.2f}%]".format(file_size_dl * 100 / file_size)
            status += chr(13)

    return filename

if __name__ == '__main__':
    filename = 'rgbd_dataset_freiburg2_pioneer_slam3.tar'
	download_file('http://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam3.tgz', renamed_file = filename)
    untar(filename)

