project('estimator', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++14'])

cmake = import('cmake')
refill = cmake.subproject('refill')

refill_lib = refill.dependency('refill')

exe = executable('estimator', 'src/estimator.cpp',
  dependencies: [refill_lib],
  install : true)

test('basic', exe)
