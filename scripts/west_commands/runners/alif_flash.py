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

    cfg_ip_file = '/build/config/app-cpu-stubs.json'
    cfg_op_file = '/build/config/tmp-cpu-stubs.json'
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
                 tool_opt=None):

        super().__init__(cfg)

        # Load J-Link defaults (raises if unavailable)
        DEFAULT_JLINK_EXE, DEFAULT_JLINK_GDB_PORT, DEFAULT_JLINK_GDB_SERVER, DEFAULT_JLINK_RTT_PORT = self._get_jlink_defaults()

        self.com_port = com_port or ''
        self.gen_toc = toc_create or 'app-gen-toc'
        self.write_toc = toc_write or 'app-write-mram'
        self.gdbserver = gdbserver or DEFAULT_JLINK_GDB_SERVER
        self.gdb_host = gdb_host or ''
        self.gdb_port = gdb_port or DEFAULT_JLINK_GDB_PORT
        self.rtt_port = rtt_port or DEFAULT_JLINK_RTT_PORT
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
        self.commander = commander or DEFAULT_JLINK_EXE
        self.loader = loader
        self.tool_opt = tool_opt or []

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
        DEFAULT_JLINK_EXE, DEFAULT_JLINK_GDB_PORT, DEFAULT_JLINK_GDB_SERVER, DEFAULT_JLINK_RTT_PORT = cls._get_jlink_defaults()

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
        parser.add_argument('--gdbserver', default=DEFAULT_JLINK_GDB_SERVER,
                            help='GDB server, default is {}'.format(
                                DEFAULT_JLINK_GDB_SERVER))
        parser.add_argument('--gdb-host', default='',
                            help='custom gdb host, defaults to the empty string '
                            'and runs a gdb server')
        parser.add_argument('--gdb-port', default=DEFAULT_JLINK_GDB_PORT,
                            help='JLink gdb port, defaults to {}'.format(
                                DEFAULT_JLINK_GDB_PORT))
        parser.add_argument('--rtt-port', default=DEFAULT_JLINK_RTT_PORT,
                            help=f'jlink rtt port, defaults to {DEFAULT_JLINK_RTT_PORT}')
        parser.add_argument('--commander', default=DEFAULT_JLINK_EXE,
                            help=f'''J-Link Commander, default is
                            {DEFAULT_JLINK_EXE}''')

        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
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
        )

    def flash(self, **kwargs):
        """Flash the binary using Alif SE tools."""

        # Resolve tool filenames (binary preferred over .py)
        self.gen_toc = self._require_tool(self.gen_toc, self.exe_dir)
        self.write_toc = self._require_tool(self.write_toc, self.exe_dir)

        fls_addr = self.flash_address_from_build_conf(self.build_conf)
        fls_size = self.build_conf.get('CONFIG_FLASH_SIZE')

        self.logger.info("Binary address %s and size %s KB", hex(fls_addr), fls_size)

        if self.build_conf.getboolean('CONFIG_RTSS_HP'):
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
            self.logger.info(f"Changed working directory to {self.exe_dir}")

            # Prepare json to create AToC.
            self.prepare_json(self.logger, build_core, fls_addr, fls_size)

            # Generate ToC.
            self.check_call(['./' + self.gen_toc, '-f',
                             self.exe_dir + self.cfg_op_file],
                            **kwargs)

            # Write ToC.
            self.check_call(['./' + self.write_toc, '--pad', '--switch'], **kwargs)

        finally:
            os.chdir(old_cwd)
            self.logger.info(f"Returned to working directory {old_cwd}")

    def _delegate_to_jlink(self, command, **kwargs):
        """Helper: create and execute J-Link runner for the given command."""
        self.logger.info(f"Delegating to JLinkBinaryRunner (command={command})...")
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

        try:
            with open(cls.exe_dir + cls.isp_cfg_file, "r") as f:
                lines = f.readlines()

            # update comport
            lines[0] = f"comport /dev/tty{com_port}\n"

            with open(cls.exe_dir + cls.isp_cfg_file, "w", newline="\n") as f:
                f.writelines(lines)

        except IOError as err:
            logger.error(f"Can't open file to read {cls.exe_dir + cls.isp_cfg_file} : {err}")
            return False

        return True

    @classmethod
    def verify_tool_config(cls, logger, device):
        """Check SE Tool configuration and device argument."""
        try:
            with open(cls.exe_dir + cls.glbl_cfg_file, 'r', encoding="utf-8") as conf_file:
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

                logger.info(f"SE Tool configured to {cfg_dev} and target {device}")
                if (cfg_dev[:5] == device[:5]):
                    return True
                else:
                    logger.error('Target and configuration Mismatch !!! '
                                 'Please re-check the Alif tools configuration !')
                    return False

        except json.JSONDecodeError as err:
            logger.error(f"Invalid JSON format: {err}")
            return False

        except IOError as err:
            logger.error(f"Can't open file to read {cls.exe_dir + cls.glbl_cfg_file} : {err}")
            return False

    @classmethod
    def get_itcm_address(cls, logger):
        """Retrieve itcm address from DTS."""
        try:
            with open(os.path.join(cls.zephyr_repo,
                                   'build', 'zephyr', 'zephyr.dts'), "r", encoding="utf-8") as f:
                dtext = f.read()
        except Exception as err:
            logger.error(f"dts parsing error {err}")
            return 0

        try:
            dt2 = fdt.parse_dts(dtext)
            addr = dt2.get_node('soc').get_subnode('itcm@0').get_property('global_base')
        except Exception as err:
            logger.error(f"Parsing itcm address {err}")
            return 0
        return addr.value

    @classmethod
    def prepare_json(cls, logger, build_core, fls_addr, fls_size):
        """Prepare JSON for ToC generation."""
        try:
            with open(cls.exe_dir + cls.cfg_ip_file, 'r', encoding="utf-8") as conf_file:
                json_data = json.load(conf_file)
        except IOError as err:
            logger.error(f"Can't open file to read {cls.exe_dir + cls.cfg_ip_file} : {err}")
            return

        if build_core == "hp":
            cpu_node = json_data["HP_APP"]
        else:
            cpu_node = json_data["HE_APP"]

        # update binary name
        cpu_node["binary"] = "zephyr.bin"

        # verify flash address
        if fls_addr == 0:
            itcm_addr = cls.get_itcm_address(logger)
            if itcm_addr == 0:
                logger.error(f"err addr 0x{itcm_addr:x}")
                return
            logger.info(f"itcm global address 0x{itcm_addr:x}")
            cpu_node["loadAddress"] = hex(itcm_addr)
            cpu_node["flags"] = ["load", "boot"]

        elif fls_addr >= cls.mram_base_addr and fls_addr <= cls.mram_base_addr + (fls_size * 1024):
            del cpu_node['loadAddress']
            cpu_node['mramAddress'] = hex(fls_addr)
            cpu_node['flags'] = ["boot"]

        else:
            raise NotImplementedError(f'Unsupported address base 0x{fls_addr:x} to write')

        try:
            with open(cls.exe_dir + cls.cfg_op_file, 'w', encoding="utf-8") as file:
                json.dump(json_data, file, indent=4)
        except IOError as err:
            logger.error(f"Can't open file to write {cls.exe_dir + cls.cfg_op_file}: {err}")

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
        )

    def _require_tool(self, tool_name: str, base_dir: str) -> str:
        """
        Ensure that required tool exists under base_dir.

        Tools are distributed in two forms: prebuilt binary or Python script.
        Alif_flash falls back in that order.
        """
        base_path = os.path.join(base_dir, tool_name)
        py_path = base_path + ".py"

        if os.path.exists(base_path):
            return tool_name

        if os.path.exists(py_path):
            self.logger.info(f"Using Python fallback for tool: {tool_name}.py")
            return tool_name + ".py"

        raise FileNotFoundError(
            f"Required tool not found in {base_dir}: '{tool_name}' or '{tool_name}.py'"
        )
