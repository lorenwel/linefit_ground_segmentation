from setuptools import setup, find_packages

setup(
    name='linefit',
    version='0.2.4',
    description='linefit ground segmentation algorithm Python bindings',
    author='Qingwen Zhang',
    author_email='qingwen@kth.se',
    url='https://github.com/Kin-Zhang/linefit',
    packages=find_packages(),
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
