pcat_sources = [
    'main.c',
    'pmu-manager.c',
    'modem-manager.c',
    'controller.c'
]

pcat_headers = [
    'common.h',
    'pmu-manager.h',
    'modem-manager.h',
    'controller.h'
]

executable('pcat-manager',
    pcat_sources,
    pcat_headers,
    install: true,
    dependencies : [
        glib2_deps,
        gthread2_deps,
        gio2_deps,
        gio2_unix_deps,
        libusb1_deps,
        jsonc_deps,
        thread_deps
    ]
)

pcat_pmu_updater_sources = [
    'pcat-pmu-updater.c'
]

pcat_pmu_updater_headers = [

]

executable('pcat-pmu-updater',
    pcat_pmu_updater_sources,
    pcat_pmu_updater_headers,
    install: true,
    dependencies : [
        glib2_deps,
        zlib_deps
    ]
)