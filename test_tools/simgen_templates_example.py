TESTBED_TEMPLATE = \
r"""
{
    "name" : "WCB Test",
    "description" : "Run WCB for {{ duration_minutes }} minutes",
    "start_time" : "{{ start_time }}",
    "duration" : {{ duration_seconds }},
    "binaries" : {
        "hardware" : "firefly",
        "bin_file":    "{{ abs_bin_path }}",
        "programAddress": "0x00200000",
        "targets":  {{ targets }}
    },
    "logs": 0,
    "orchestrator" : {
        "type" : "python",
        "file" : "TO BE MODIFIED: (absolute) path to tcp_orchestrator.py",
        "init" : "init_test",
        "init_kargs" : {
            "scenario" : "{{ version }}",
            "seed" : {{ seed }}
        },
        "run" : "run_test"
    },
    "extra_files" : "TO BE MODIFIED: (absolute) path to the data.mat file were several control variables (e.g., K, Nt, Qt, ...) are defined"
}
"""