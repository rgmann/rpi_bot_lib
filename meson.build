project(
    'rpi_bot_lib', 'cpp',
    meson_version : '>= 0.63',
    version : '1.0.0',
    default_options : [
        # 'werror=true',
        'warning_level=2',
        'cpp_std=c++17',
    ],
    license : 'LGPL 2.1+'
)

include_root = 'include'

sources = [
    'source/adxl345_controller.cpp',
    'source/i2c_interface.cpp',
    'source/pwm_controller.cpp'
]

common_arguments = [
    '-Wmissing-declarations',
    '-Wshadow',
    # '-include', meson.current_build_dir() / 'config.h',
]

c_arguments = []
cpp_arguments = []

# cc = meson.get_compiler('c')
cxx = meson.get_compiler('cpp')
# config_h = configuration_data()

if cxx.get_id() == 'clang'
    if cxx.version().version_compare('<9')
        error('clang version is too old, rpi_bot_lib requires 9.0 or newer')
    endif

    # Turn _FORTIFY_SOURCE by default on. This is needed on clang only as gcc
    # enables it by default. FORTIFY will not work properly with `-O0`, and may
    # result in macro redefinition errors if the user already has a setting for
    # `-D_FORTIFY_SOURCE`. Do not enable FORTIFY in either of those cases.
    if get_option('optimization') != '0'
        fortify = cxx.get_define('_FORTIFY_SOURCE')
        if fortify == ''
            message('Adding _FORTIFY_SOURCE')
            common_arguments += [
                '-D_FORTIFY_SOURCE=2',
            ]
        endif
    endif

    # Use libc++ by default if available instead of libstdc++ when compiling
    # with clang.
    if cxx.find_library('c++', required : false).found()
        cpp_arguments += [
            '-stdlib=libc++',
        ]
    endif

    cpp_arguments += [
        #'-Wextra-semi',
        '-Wthread-safety',
    ]
endif

if cxx.get_id() == 'gcc'
    if cxx.version().version_compare('<9')
        error('gcc version is too old, rpi_bot_lib requires 9.0 or newer')
    endif

    # gcc 13 implements the C++23 version of automatic move from local
    # variables in return statements (see
    # https://en.cppreference.com/w/cpp/language/return). As a result, some
    # previously required explicit std::move() in return statements generate
    # warnings. Those moves can't be removed as older compiler versions could
    # use copy constructors instead of move constructors. The easiest fix is to
    # disable the warning. With -Wpessimizing-move enabled, the compiler will
    # still warn of pessimizing moves, only the redundant but not pessimizing
    # moves will be ignored.
    if cxx.version().version_compare('>=13')
        cpp_arguments += [
            '-Wno-redundant-move',
        ]
    endif

    # gcc 7.1 introduced processor-specific ABI breakages related to parameter
    # passing on ARM platforms. This generates a large number of messages
    # during compilation. Silence them.
    if host_machine.cpu_family() == 'arm'
        cpp_arguments += [
            '-Wno-psabi',
        ]
    endif
endif

c_arguments += common_arguments
cpp_arguments += common_arguments

add_project_arguments(c_arguments, language : 'c')
add_project_arguments(cpp_arguments, language : 'cpp')
add_project_link_arguments(cpp_arguments, language : 'cpp')

shared_library(
    'rpibotlib', sources,
    cpp_args: cpp_arguments,
    include_directories : [ include_root ]
)