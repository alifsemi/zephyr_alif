# Copyright 2025 AlifSemiconductor.
# SPDX-License-Identifier: Apache-2.0
"""
Runner for Alif binary image burner with J-Link debug delegation.
"""
import os
import sys
import json
import shutil
import re
import fdt

from pathlib import Path
from runners.core import ZephyrBinaryRunner, RunnerCaps, FileType
from runners.jlink import JLinkBinaryRunner


class AlifImageBinaryRunner(ZephyrBinaryRunner):
    """Runner front-end for Alif Image Flasher with J-Link delegation."""

    zephyr_repo = str(Path.cwd())
    exe_dir = os.getenv("ALIF_SE_TOOLS_DIR")

    mram_base_addr = int('0x80000000', 0)
    default_flash_addresses = {
        "A32_APP": int('0x80100000', 0),
        "HP_APP": int('0x80200000', 0),
        "HE_APP": int('0x80000000', 0),
    }
    bl32_addr_def = int('0x80002000', 0)
    cfg_ip_file = '/build/config/app-cpu-stubs.json'
    cfg_op_file = '/build/config/west-gen-flash-conf.json'
    glbl_cfg_file = '/utils/global-cfg.db'
    isp_cfg_file = '/isp_config_data.cfg'

    def __init__(self, cfg, device,
                 dev_id=None,
                 commander=None,
                 erase=None,
                 reset=None,
                 iface=None,
                 speed=None,
                 loader=None,
                 gdbserver=None,
                 gdb_host=None,
                 gdb_port=None,
                 rtt_port=None,
                 tui=None,
                 com_port=None,
                 toc_create=None,
                 toc_write=None,
                 tool_opt=None,
                 a32_bin=None,
                 a32_flash_address=None,
                 a32_mram_boot=None,
                 hp_bin=None,
                 hp_flash_address=None,
                 hp_mram_boot=None,
                 he_bin=None,
                 he_flash_address=None,
                 he_mram_boot=None,
                 bl32_bin=None,
                 bl32_flash_address=None,
                 flash_address=None):

        super().__init__(cfg)

        # Load J-Link defaults (raises if unavailable)
        default_jlink_exe, default_jlink_gdb_port, default_jlink_gdb_server, default_jlink_rtt_port = self._get_jlink_defaults()

        # Load ALIF tool defaults
        default_gen_toc, default_write_toc = self._get_alif_tool_defaults()

        self.com_port = com_port or ''
        self.gen_toc = toc_create or default_gen_toc
        self.write_toc = toc_write or default_write_toc
        self.gdbserver = gdbserver or default_jlink_gdb_server
        self.gdb_host = gdb_host or ''
        self.gdb_port = gdb_port or default_jlink_gdb_port
        self.rtt_port = rtt_port or default_jlink_rtt_port
        self.device = device
        self.iface = iface or 'swd'
        self.speed = speed or 'auto'
        self.erase = bool(erase) if erase is not None else False
        self.reset = bool(reset) if reset is not None else True
        self.gdb_cmd = [cfg.gdb] if cfg.gdb else None
        self.file = cfg.file
        self.file_type = cfg.file_type
        self.hex_name = cfg.hex_file
        self.elf_name = cfg.elf_file
        self.tui_arg = ['-tui'] if tui else []

        self.dev_id = dev_id
        self.commander = commander or default_jlink_exe
        self.loader = loader
        self.tool_opt = tool_opt or []
        self.a32_bin = os.path.abspath(a32_bin) if a32_bin else None
        self.a32_flash_address = a32_flash_address
        self.a32_mram_boot = False if a32_mram_boot is None else bool(a32_mram_boot)
        self.hp_bin = os.path.abspath(hp_bin) if hp_bin else None
        self.hp_flash_address = hp_flash_address
        self.hp_mram_boot = False if hp_mram_boot is None else bool(hp_mram_boot)
        self.he_bin = os.path.abspath(he_bin) if he_bin else None
        self.he_flash_address = he_flash_address
        self.he_mram_boot = False if he_mram_boot is None else bool(he_mram_boot)
        self.bl32_bin = os.path.abspath(bl32_bin) if bl32_bin else None
        self.bl32_flash_address = (bl32_flash_address
                                   if bl32_flash_address is not None
                                   else self.bl32_addr_def)
        self.flash_address = flash_address or []

    @classmethod
    def name(cls):
        return 'alif_flash'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash', 'debug', 'debugserver', 'attach'},
                          dev_id=True, erase=True, reset=True,
                          tool_opt=True, file=True)

    @classmethod
    def do_add_parser(cls, parser):
        # Load J-Link defaults (raises if unavailable)
        default_jlink_exe, default_jlink_gdb_port, default_jlink_gdb_server, default_jlink_rtt_port = cls._get_jlink_defaults()

        # Required:
        parser.add_argument('--device', required=True, help='device name')

        # Optional:
        parser.add_argument('--loader', required=False, dest='loader',
                            help='specifies a loader type')
        parser.add_argument('--com-port', default='',
                            help='SE Port to write ToC image')
        parser.add_argument('--iface', default='swd',
                            help='interface to use, default is swd')
        parser.add_argument('--speed', default='auto',
                            help='interface speed, default is autodetect')
        parser.add_argument('--tui', default=False, action='store_true',
                            help='if given, GDB uses -tui')
        parser.add_argument('--gdbserver', default=default_jlink_gdb_server,
                            help='GDB server, default is {}'.format(
                                default_jlink_gdb_server))
        parser.add_argument('--gdb-host', default='',
                            help='custom gdb host, defaults to the empty string '
                            'and runs a gdb server')
        parser.add_argument('--gdb-port', default=default_jlink_gdb_port,
                            help='JLink gdb port, defaults to {}'.format(
                                default_jlink_gdb_port))
        parser.add_argument('--rtt-port', default=default_jlink_rtt_port,
                            help=f'jlink rtt port, defaults to {default_jlink_rtt_port}')
        parser.add_argument('--commander', default=default_jlink_exe,
                            help=f'''J-Link Commander, default is
                            {default_jlink_exe}''')
        parser.add_argument('--a32-bin', help='A32 application binary to include in ToC')
        parser.add_argument('--a32-flash-address', type=lambda x: int(x, 0),
                            help='MRAM address for --a32-bin, default is 0x80100000')
        parser.add_argument('--a32-mram-boot', '-a32-mram-boot',
                            default=0, type=int, choices=(0, 1),
                            help='set to 1 to boot --a32-bin from MRAM')
        parser.add_argument('--hp-bin', help='M55 HP application binary to include in ToC')
        parser.add_argument('--hp-flash-address', type=lambda x: int(x, 0),
                            help='MRAM address for --hp-bin, default is 0x80200000')
        parser.add_argument('--hp-mram-boot', '-hp-mram-boot',
                            default=0, type=int, choices=(0, 1),
                            help='set to 1 to boot --hp-bin from MRAM')
        parser.add_argument('--he-bin', help='M55 HE application binary to include in ToC')
        parser.add_argument('--he-flash-address', type=lambda x: int(x, 0),
                            help='MRAM address for --he-bin, default is 0x80000000')
        parser.add_argument('--he-mram-boot', '-he-mram-boot',
                            default=0, type=int, choices=(0, 1),
                            help='set to 1 to boot --he-bin from MRAM')
        parser.add_argument('--bl32-bin', help='BL32 bootloader binary, required for A32 builds')
        parser.add_argument('--bl32-flash-address', type=lambda x: int(x, 0),
                            default=cls.bl32_addr_def,
                            help='MRAM address for BL32 bootloader in A32 builds; '
                                 'used with --bl32-bin, default is 0x80002000')
        parser.add_argument('--flash-address', action='append', type=lambda x: int(x, 0),
                            help='MRAM address for supplied CPU binaries in A32, HP, HE order')

        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        cls._validate_cpu_bin_args(args)
        return AlifImageBinaryRunner(
            cfg,
            device=args.device,
            dev_id=getattr(args, "dev_id", None),
            commander=getattr(args, "commander", None),
            erase=getattr(args, "erase", None),
            reset=getattr(args, "reset", None),
            iface=getattr(args, "iface", None),
            speed=getattr(args, "speed", None),
            loader=getattr(args, "loader", None),
            gdbserver=getattr(args, "gdbserver", None),
            gdb_host=getattr(args, "gdb_host", None),
            gdb_port=getattr(args, "gdb_port", None),
            rtt_port=getattr(args, "rtt_port", None),
            tui=getattr(args, "tui", None),
            com_port=getattr(args, "com_port", None),
            toc_create=getattr(args, "toc_create", None),
            toc_write=getattr(args, "toc_write", None),
            tool_opt=getattr(args, "tool_opt", None),
            a32_bin=getattr(args, "a32_bin", None),
            a32_flash_address=getattr(args, "a32_flash_address", None),
            a32_mram_boot=getattr(args, "a32_mram_boot", None),
            hp_bin=getattr(args, "hp_bin", None),
            hp_flash_address=getattr(args, "hp_flash_address", None),
            hp_mram_boot=getattr(args, "hp_mram_boot", None),
            he_bin=getattr(args, "he_bin", None),
            he_flash_address=getattr(args, "he_flash_address", None),
            he_mram_boot=getattr(args, "he_mram_boot", None),
            bl32_bin=getattr(args, "bl32_bin", None),
            bl32_flash_address=getattr(args, "bl32_flash_address", None),
            flash_address=getattr(args, "flash_address", None),
        )

    @staticmethod
    def _validate_cpu_bin_args(args):
        for cpu in ("a32", "hp", "he"):
            binary = getattr(args, f"{cpu}_bin", None)
            address = getattr(args, f"{cpu}_flash_address", None)

            if binary is None and address is not None:
                raise ValueError(f"--{cpu}-flash-address requires --{cpu}-bin")

        # --bl32-flash-address has a default and is used only with --bl32-bin.

    def flash(self, **kwargs):
        """Flash the binary using Alif SE tools."""

        # Resolve tool filenames (binary preferred over .py)
        self.gen_toc = self._require_tool(self.gen_toc, self.exe_dir)
        self.write_toc = self._require_tool(self.write_toc, self.exe_dir)

        fls_addr = self.flash_address_from_build_conf(self.build_conf)
        fls_size = self.build_conf.get('CONFIG_FLASH_SIZE')

        self.logger.info("Binary address %s and size %s KB", hex(fls_addr), fls_size)

        if self.build_conf.getboolean('CONFIG_APSS'):
            self.logger.info("..build for APSS Core")
            build_core = "a32"
        elif self.build_conf.getboolean('CONFIG_RTSS_HP'):
            self.logger.info("..build for HighPerformance Core")
            build_core = "hp"
        else:
            self.logger.info("..build for HighEfficency Core")
            build_core = "he"

        shutil.copy(
            os.path.join(self.cfg.build_dir, 'zephyr', 'zephyr.bin'),
            os.path.join(self.exe_dir, 'build/images/')
        )

        # change dir just for the JSON and TOC
        old_cwd = os.getcwd()
        try:
            os.chdir(self.exe_dir)
            self.logger.info("Changed working directory to %s", self.exe_dir)

            # Prepare json to create AToC.
            if not self.prepare_json(self.logger, build_core, fls_addr, fls_size,
                                     self.a32_bin, self.a32_flash_address,
                                     self.a32_mram_boot,
                                     self.hp_bin, self.hp_flash_address,
                                     self.hp_mram_boot,
                                     self.he_bin, self.he_flash_address,
                                     self.he_mram_boot,
                                     self.bl32_bin,
                                     self.bl32_flash_address,
                                     self.flash_address,
                                     self.cfg.build_dir):
                return

            # Generate ToC.
            self.check_call(['./' + self.gen_toc, '-f',
                             self.exe_dir + self.cfg_op_file],
                            **kwargs)

            # Write ToC.
            self.check_call(['./' + self.write_toc, '-p'], **kwargs)

        finally:
            os.chdir(old_cwd)
            self.logger.info("Returned to working directory %s", old_cwd)

    def _delegate_to_jlink(self, command, **kwargs):
        """Helper: create and execute J-Link runner for the given command."""
        self.logger.info("Delegating to JLinkBinaryRunner (command=%s)...", command)
        jlink_runner = self._create_jlink_runner()
        jlink_runner.do_run(command, **kwargs)

    def do_run(self, command, **kwargs):
        """Entry point for west runner commands."""
        # check env
        if self.exe_dir is None:
            raise RuntimeError("ALIF_SE_TOOLS_DIR Environment unset")

        # Attach : Launch debug session to the running process.
        if command == 'attach':
            self._delegate_to_jlink(command, **kwargs)
            return

        # Verification: SE Configuration with Target
        if not self.verify_tool_config(self.logger, self.device):
            return

        # Update SE communication port.
        if not self.verify_se_port(self.logger, self.com_port):
            self.logger.warning("Failed to update COMPORT")

        # Flash : Write to MRAM with Alif SE Tools.
        if command == 'flash':
            self.flash(**kwargs)
            return

        # Debug : Flash and launch JLink debug session.
        if command == 'debug':
            self.flash(**kwargs)
            self._delegate_to_jlink(command, **kwargs)
            return

        # Debug server: start JLink GDB server only (no flashing)
        if command == 'debugserver':
            self._delegate_to_jlink(command, **kwargs)
            return

    @classmethod
    def verify_se_port(cls, logger, com_port):
        """Verify and update Secure Enclave communication port."""

        if not com_port:
            return True

        isp_cfg_path = os.path.join(cls.exe_dir, cls.isp_cfg_file.lstrip('/'))

        try:
            with open(isp_cfg_path, "r", encoding="utf-8") as f:
                lines = f.readlines()

            # update comport
            if sys.platform.startswith("win"):
                lines[0] = f"comport {com_port}\n"
            else:
                lines[0] = f"comport /dev/tty{com_port}\n"

            with open(isp_cfg_path, "w", encoding="utf-8", newline="\n") as f:
                f.writelines(lines)

        except OSError as err:
            logger.error("Can't open file to read %s: %s", isp_cfg_path, err)
            return False

        return True

    @classmethod
    def verify_tool_config(cls, logger, device):
        """Check SE Tool configuration and device argument."""
        glbl_cfg_path = os.path.join(cls.exe_dir, cls.glbl_cfg_file.lstrip('/'))

        try:
            with open(glbl_cfg_path, 'r', encoding="utf-8") as conf_file:
                json_data = json.load(conf_file)
                value = json_data.get("DEVICE", {}).get("Part#")

                if not value:
                    logger.error("Failed to fetch Part config")
                    return False

                dev_val = re.search(r"\(([^)]+)\)", value)
                if dev_val:
                    cfg_dev = dev_val.group(1)
                else:
                    logger.error("Failed to fetch Device config")
                    return False

                logger.info("SE Tool configured to %s and target %s", cfg_dev, device)
                if (cfg_dev[:5] == device[:5]):
                    return True
                else:
                    logger.error('Target and configuration Mismatch !!! '
                                  'Please re-check the Alif tools configuration!')
                    return False

        except json.JSONDecodeError as err:
            logger.error("Invalid JSON format: %s", err)
            return False

        except OSError as err:
            logger.error("Can't open file to read %s: %s", glbl_cfg_path, err)
            return False

    @classmethod
    def get_itcm_address(cls, logger, build_dir=None):
        """Retrieve itcm address from DTS."""
        dt2 = cls._parse_build_dts(logger, build_dir)
        if dt2 is None:
            return 0

        try:
            itcm = cls._get_node_by_label(dt2, 'itcm')
            addr = cls._dts_property_value(itcm, 'global_base')
            if addr is None:
                raise ValueError("DTS itcm global_base property not found")
            return addr
        except (AttributeError, KeyError, TypeError, ValueError) as err:
            logger.error("Parsing itcm address failed: %s", err)
        return 0

    @classmethod
    def get_sram0_address(cls, logger, build_dir=None):
        """Retrieve sram0 base address from DTS reg property."""
        dt2 = cls._parse_build_dts(logger, build_dir)
        if dt2 is None:
            return 0

        try:
            sram0 = cls._get_node_by_label(dt2, 'sram0')
            reg = cls._dts_property_value(sram0, 'reg')
            if reg is None:
                raise ValueError("DTS sram0 reg property not found")
            return reg
        except (AttributeError, KeyError, TypeError, ValueError) as err:
            logger.error("Parsing sram0 address failed: %s", err)
        return 0

    @classmethod
    def _parse_build_dts(cls, logger, build_dir=None):
        """Parse generated zephyr.dts using fdt."""
        if build_dir is None:
            build_dir = os.path.join(cls.zephyr_repo, 'build')

        dts_path = os.path.join(build_dir, 'zephyr', 'zephyr.dts')

        try:
            with open(dts_path, "r", encoding="utf-8") as f:
                dtext = f.read()
        except OSError as err:
            logger.error("DTS read error (%s): %s", dts_path, err)
            return None

        try:
            return fdt.parse_dts(dtext)
        except (AttributeError, TypeError, ValueError) as err:
            logger.error("DTS parse error (%s): %s", dts_path, err)
            return None

    @staticmethod
    def _get_node_by_label(dt2, label):
        getter = getattr(dt2, 'get_node_by_label', None)
        if callable(getter):
            node = getter(label)
            if node is not None:
                return node

        def find_labeled_node(node):
            if getattr(node, 'label', None) == label:
                return node

            for child in getattr(node, 'nodes', []):
                found = find_labeled_node(child)
                if found is not None:
                    return found

            return None

        root = dt2.get_node('/')
        node = find_labeled_node(root)
        if node is not None:
            return node

        raise ValueError(f"DTS node label {label} not found")

    @staticmethod
    def _dts_property_value(node, name):
        prop = node.get_property(name)
        if prop is None:
            return None

        value = getattr(prop, 'value', prop)
        if isinstance(value, list):
            return value[0] if value else None
        return value

    @classmethod
    def get_load_address(cls, logger, node_name, build_dir=None):
        """Retrieve load address for local non-MRAM boot image."""
        if node_name in ("HP_APP", "HE_APP"):
            return cls.get_itcm_address(logger, build_dir)
        return cls.get_sram0_address(logger, build_dir)

    @classmethod
    def prepare_json(cls, logger, build_core, fls_addr, fls_size,
                     a32_bin=None, a32_flash_address=None,
                     a32_mram_boot=False,
                     hp_bin=None, hp_flash_address=None,
                     hp_mram_boot=False,
                     he_bin=None, he_flash_address=None,
                     he_mram_boot=False,
                     bl32_bin=None,
                     bl32_flash_address=bl32_addr_def,
                     flash_address=None,
                     build_dir=None):
        """Prepare JSON for ToC generation."""
        cfg_ip_path = os.path.join(cls.exe_dir, cls.cfg_ip_file.lstrip('/'))
        cfg_op_path = os.path.join(cls.exe_dir, cls.cfg_op_file.lstrip('/'))

        try:
            with open(cfg_ip_path, 'r', encoding="utf-8") as conf_file:
                json_data = json.load(conf_file)
        except OSError as err:
            logger.error("Can't open file to read %s: %s", cfg_ip_path, err)
            return False

        local_node_name = cls._cpu_node_name(build_core)

        mram_ranges = []

        if fls_addr != 0 and fls_addr >= cls.mram_base_addr:
            local_bin = os.path.join(cls.exe_dir, 'build/images', 'zephyr.bin')
            local_size = cls._binary_size(local_node_name, local_bin)

            cls._add_mram_range(local_node_name, fls_addr,
                                fls_addr + local_size, fls_size,
                                mram_ranges)

        cpu_args = [
            ("A32_APP", a32_bin, a32_flash_address, a32_mram_boot),
            ("HP_APP", hp_bin, hp_flash_address, hp_mram_boot),
            ("HE_APP", he_bin, he_flash_address, he_mram_boot),
        ]
        flash_addresses = list(flash_address or [])

        for node_name, binary, address, mram_boot in cpu_args:
            if binary is None:
                continue

            if node_name == local_node_name:
                logger.info("Ignoring %s argument; local build overwrites this CPU",
                            node_name)
                continue

            if mram_boot:
                address = cls._cpu_flash_address(node_name, address, flash_addresses)
            elif address is None and flash_addresses:
                flash_addresses.pop(0)

            if not cls._apply_cpu_arg_image(logger, json_data, node_name,
                                            binary, address, fls_size,
                                            mram_ranges, mram_boot):
                return False

        if not cls._apply_local_build_image(logger, json_data, local_node_name,
                                            fls_addr, fls_size, build_dir):
            return False

        if build_core == "a32" or a32_bin is not None:
            cls._validate_a32_bootload_args(bl32_bin, bl32_flash_address)
            if not cls._apply_a32_bootload_node(json_data, logger, bl32_bin,
                                                bl32_flash_address, fls_size,
                                                mram_ranges):
                return False

        try:
            with open(cfg_op_path, 'w', encoding="utf-8") as file:
                json.dump(json_data, file, indent=4)
        except OSError as err:
            logger.error("Can't open file to write %s: %s", cfg_op_path, err)
            return False

        return True

    @classmethod
    def _validate_a32_bootload_args(cls, bl32_bin, bl32_flash_address):
        if bl32_bin is None:
            raise ValueError("A32 image requires --bl32-bin")

    @classmethod
    def _apply_a32_bootload_node(cls, json_data, logger, bl32_bin=None,
                                 bl32_flash_address=None, fls_size=None,
                                 mram_ranges=None):
        binary = os.path.basename(bl32_bin)
        address = bl32_flash_address

        try:
            bin_size = cls._binary_size("BOOTLOAD", bl32_bin)
        except FileNotFoundError as err:
            logger.error("%s", err)
            return False

        cls._add_mram_range("BOOTLOAD", address, address + bin_size,
                            fls_size, mram_ranges or [])
        image_dir = os.path.join(cls.exe_dir, 'build/images')
        try:
            shutil.copy(bl32_bin, image_dir)
        except OSError as err:
            logger.error("Can't copy BOOTLOAD binary %s: %s", bl32_bin, err)
            return False

        bootload_node = {
            "binary": binary,
            "version": "0.4.3",
            "mramAddress": hex(address),
            "signed": True,
            "cpu_id": "A32_0",
            "flags": ["boot"],
        }

        if "BOOTLOAD" in json_data:
            json_data["BOOTLOAD"] = bootload_node
            return True

        if "A32_APP" not in json_data:
            json_data["BOOTLOAD"] = bootload_node
            return True

        ordered_data = {}
        for key, value in json_data.items():
            if key == "A32_APP":
                ordered_data["BOOTLOAD"] = bootload_node
            ordered_data[key] = value

        json_data.clear()
        json_data.update(ordered_data)
        return True

    @staticmethod
    def _cpu_node_name(build_core):
        if build_core == "a32":
            return "A32_APP"
        if build_core == "hp":
            return "HP_APP"
        return "HE_APP"

    @classmethod
    def _cpu_flash_address(cls, node_name, address, flash_addresses):
        if address is not None:
            return address

        if flash_addresses:
            return flash_addresses.pop(0)

        return cls.default_flash_addresses[node_name]

    @classmethod
    def _binary_size(cls, node_name, binary):
        try:
            return os.path.getsize(binary)
        except OSError as err:
            raise FileNotFoundError(f'{node_name} binary not found: {binary}') from err

    @classmethod
    def _mram_limit(cls, fls_size):
        if fls_size is None:
            return None
        return cls.mram_base_addr + (fls_size * 1024)

    @classmethod
    def _validate_mram_boundary(cls, node_name, start, end, fls_size):
        if start < cls.mram_base_addr:
            raise ValueError(f'Unsupported address base 0x{start:x} to write')

        mram_limit = cls._mram_limit(fls_size)
        if mram_limit is not None and end > mram_limit:
            raise ValueError(
                f'{node_name} binary range 0x{start:x}-0x{end:x} '
                f'exceeds MRAM limit 0x{mram_limit:x}')

    @staticmethod
    def _validate_mram_overlap(node_name, start, end, mram_ranges):
        for other_node, other_start, other_end in mram_ranges:
            if start < other_end and other_start < end:
                raise ValueError(
                    f'{node_name} binary range 0x{start:x}-0x{end:x} '
                    f'overlaps {other_node} range 0x{other_start:x}-0x{other_end:x}')

    @classmethod
    def _add_mram_range(cls, node_name, start, end, fls_size, mram_ranges):
        cls._validate_mram_boundary(node_name, start, end, fls_size)
        cls._validate_mram_overlap(node_name, start, end, mram_ranges)
        mram_ranges.append((node_name, start, end))

    @classmethod
    def _apply_cpu_arg_image(cls, logger, json_data, node_name, binary, address,
                             fls_size, mram_ranges, mram_boot=True):
        try:
            bin_size = cls._binary_size(node_name, binary)
        except FileNotFoundError as err:
            logger.error("%s", err)
            return False

        if node_name not in json_data:
            logger.error("%s binary was provided, but %s is not supported by "
                         "the selected part configuration",
                         node_name, node_name)
            return False

        if mram_boot:
            cls._add_mram_range(node_name, address, address + bin_size, fls_size,
                                mram_ranges)

        image_dir = os.path.join(cls.exe_dir, 'build/images')
        try:
            shutil.copy(binary, image_dir)
        except OSError as err:
            logger.error("Can't copy %s binary %s: %s", node_name, binary, err)
            return False

        cpu_node = json_data[node_name]
        cpu_node["binary"] = os.path.basename(binary)
        if not mram_boot:
            if node_name == "A32_APP":
                cpu_node.pop('flags', None)
                cpu_node.pop('loadAddress', None)
            logger.info("Updated %s binary %s with existing load configuration",
                        node_name, cpu_node["binary"])
            return True

        cpu_node.pop('loadAddress', None)
        cpu_node["mramAddress"] = hex(address)
        if node_name == "A32_APP":
            cpu_node.pop('flags', None)
        else:
            cpu_node["flags"] = ["boot"]
        logger.info("Updated %s binary %s at %s size %d bytes",
                    node_name, cpu_node["binary"], cpu_node["mramAddress"],
                    bin_size)
        return True

    @classmethod
    def _apply_local_build_image(cls, logger, json_data, node_name, fls_addr,
                                 fls_size, build_dir=None):
        if node_name not in json_data:
            logger.error("%s local build is not supported by the selected "
                         "part configuration", node_name)
            return False

        cpu_node = json_data[node_name]

        # update binary name
        cpu_node["binary"] = "zephyr.bin"

        # verify flash address
        if fls_addr == 0 :
            load_addr = cls.get_load_address(logger, node_name, build_dir)
            if load_addr == 0:
                logger.error("err addr 0x%x", load_addr)
                return False
            logger.info("%s load address 0x%x", node_name, load_addr)
            cpu_node["loadAddress"] = hex(load_addr)
            if node_name == "A32_APP":
                cpu_node.pop('flags', None)
            else:
                cpu_node["flags"] = ["load", "boot"]
            return True

        mram_limit = cls._mram_limit(fls_size)
        if fls_addr >= cls.mram_base_addr and (
                mram_limit is None or fls_addr <= mram_limit):
            cpu_node.pop('loadAddress', None)
            cpu_node['mramAddress'] = hex(fls_addr)
            if node_name == "A32_APP":
                cpu_node.pop('flags', None)
            else:
                cpu_node['flags'] = ["boot"]
            return True

        raise NotImplementedError(f'Unsupported address base 0x{fls_addr:x} to write')

    @staticmethod
    def _get_jlink_defaults():
        """Import J-Link defaults, or fail loudly if J-Link runner not available."""
        from runners import jlink
        return (
            jlink.DEFAULT_JLINK_EXE,
            jlink.DEFAULT_JLINK_GDB_PORT,
            jlink.DEFAULT_JLINK_GDB_SERVER,
            jlink.DEFAULT_JLINK_RTT_PORT
        )

    def _create_jlink_runner(self):
        """Helper: instantiate a JLinkBinaryRunner with Alif's parameters."""
        return JLinkBinaryRunner(
            self.cfg,
            device=self.device,
            dev_id=self.dev_id,
            commander=self.commander,
            dt_flash=True,
            erase=self.erase,
            reset=self.reset,
            iface=self.iface,
            speed=self.speed,
            loader=self.loader,
            gdbserver=self.gdbserver,
            gdb_host=self.gdb_host,
            gdb_port=self.gdb_port,
            tui=len(self.tui_arg) > 0,
            tool_opt=self.tool_opt,
            rtt_port=self.rtt_port,
        )

    @staticmethod
    def _get_alif_tool_defaults():
        """
        Return default ALIF SE tool names.

        Platform-specific executable suffix handling is done here,
        similar to J-Link defaults.
        """
        if sys.platform.startswith(("win", "msys", "cygwin")):
            return (
                "app-gen-toc.exe",
                "app-write-mram.exe",
            )

        return (
            "app-gen-toc",
            "app-write-mram",
        )

    def _require_tool(self, tool_name: str, base_dir: str) -> str:
        """
        Ensure that required tool exists under base_dir.

        The tool can be provided as:
        - native executable (tool_name)
        - Python fallback script (<base_name>.py)

        Platform-specific executable resolution is handled in CTOR default values by calling _get_alif_tool_defaults
        """
        base_name = os.path.splitext(os.path.basename(tool_name))[0]

        candidates = [
            os.path.join(base_dir, tool_name),
            os.path.join(base_dir, base_name + ".py"),
        ]

        for path in candidates:
            if os.path.exists(path):
                chosen = os.path.basename(path)
                self.logger.info("Using tool: %s", chosen)
                return chosen

        tried = ", ".join(os.path.basename(p) for p in candidates)

        msg = "Required tool not found in '%s'. Tried: %s"
        self.logger.error(msg, base_dir, tried)
        raise FileNotFoundError(msg % (base_dir, tried))
