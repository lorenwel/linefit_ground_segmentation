from setuptools import setup, find_packages

setup(
    name='dufomap',
    version='0.3.0',
    description='DUFOMap Python bindings',
    author='Qingwen Zhang, Daniel Duberg',
    author_email='qzhangcb@connect.ust.hk',
    url='https://github.com/yourusername/dufomap',
    packages=find_packages(),
    # package_data={
    #     'dufomap': ['build/libdm_lib.so'],  # Replace 'lib/' with the actual relative path within your package
    #     'glog': ['build/glog/libglog.so'],
    # },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
    ],
    install_requires=[
        # List your package dependencies here
    ],
)
