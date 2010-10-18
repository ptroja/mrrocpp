# This script performs basic setup of QNX for use with MRROC++.
# If you want to write something to the console, use info(), warn() and err().
# Using print will mess the files we are editing inplace using fileinput.
# Warning! This script is not completely foolproof!

import fileinput, getpass, re, os, os.path, sys, urllib

def info(msg):
    sys.stderr.write(msg + "\n")

def warn(msg):
    sys.stderr.write("* " + msg + "\n")

def err(msg):
    sys.stderr.write("!!!! " + msg + "\n")

def bname(str):
    return str[str.rfind("/") + 1:]

def enable_cfg_file_entry(file_name, entry_name, entry_regex):
    cfg_file = fileinput.input(file_name, inplace=1)
    entry_regex = r"\s*" + entry_regex
    cmt_entry_regex = r"#\s*" + entry_regex
    entry_enabled = False
    for line in cfg_file:
        if re.match(entry_regex, line):
            warn("%s already enabled in %s" % (entry_name, file_name))
            entry_enabled = True
        elif re.match(cmt_entry_regex, line):
            info("Enabling %s in %s" % (entry_name, file_name))
            line = line.replace("#", "").lstrip()
            entry_enabled = True
        sys.stdout.write(line)
    if not entry_enabled:
        err("Failed to enable %s in %s" % (entry_name, file_name))
    cfg_file.close()

def enable_inetd_tcp4_service(file_name, service):
    inetd_conf_file = fileinput.input(file_name, inplace=1)
    service_regex = service + r"\s+stream\s+tcp\s+nowait"
    enable_cfg_file_entry(file_name, service, service_regex)

# Always supply full path of the service, i.e. /usr/sbin/sshd
def enable_rc_service(file_name, service):
    service_enabled = False
    rc_file = file(file_name)
    base_service = bname(service)
    service_regex = r"\s*" + service
    base_service_regex = r"\s*" + base_service
    for line in rc_file:
        if re.match(service, line) or re.match(base_service, line):
            warn("%s already enabled in %s" % (base_service, file_name))
            service_enabled = True
            break
    if not service_enabled:
        info("Enabling %s in %s" %(base_service, file_name))
        rc_file.close()
        rc_file = file(file_name, "a")
        rc_file.write(os.linesep.join((service, "")))
    rc_file.close()

def add_cfg_file_entry(file_name, entry_name, entry, match_expr):
    cfg_file = file(file_name)
    entry_found = False
    for line in cfg_file:
        if re.match(match_expr, line):
            warn("entry '%s' already present in %s" % (entry_name, file_name))
            entry_found = True
            break
    if not entry_found:
        info("Adding entry '%s' to %s" % (entry_name, file_name))
        cfg_file.close()
        cfg_file = file(file_name, "a")
        cfg_file.write(os.linesep.join((entry, "")))
    cfg_file.close()


def urlretrieve_hook(blocks_transferred, block_size, total_size):
    percent_complete = (blocks_transferred * block_size * 100) / total_size
    if (percent_complete % 5 == 0):
        sys.stderr.write("*")

# ========== The script starts here =========
info("********** QNX setup for MRROC++ **********")

# Activate QNET
if os.path.exists(r"/etc/system/config/useqnet"):
    warn("QNET already activated")
else:
    info("Activating QNET...")
    os.system(r"touch /etc/system/config/useqnet")

# Enable inetd services
inetd_conf_file_name = r"/etc/inetd.conf"
for service in "shell", "login", "phrelay":
    enable_inetd_tcp4_service(inetd_conf_file_name, service)

# Enable inetd and ssh in rc.local
rc_local_file_name = "/etc/rc.d/rc.local"
for service in "/usr/sbin/inetd", "/usr/sbin/sshd":
    enable_rc_service(rc_local_file_name, service)
# Add pkgsrc libpath in rc.local
add_cfg_file_entry(rc_local_file_name, "CS_LIBPATH for pkgsrc",
  r"setconf CS_LIBPATH $(getconf CS_LIBPATH):/usr/pkg/lib",
  r"setconf\s+CS_LIBPATH\s+\$\(getconf\s+CS_LIBPATH\):\/usr\/pkg\/lib")

# Generate sshd keys
dsa_key_file_name = r"/etc/ssh/ssh_host_dsa_key"
rsa_key_file_name = r"/etc/ssh/ssh_host_rsa_key"
if os.path.exists(dsa_key_file_name):
    warn("dsa host key for sshd already exists")
else:
    os.system('/usr/bin/ssh-keygen -f "%s" -t dsa -N ""' % dsa_key_file_name)
if os.path.exists(rsa_key_file_name):
    warn("rsa host key for sshd already exists")
else:
    os.system('/usr/bin/ssh-keygen -f "%s" -t rsa -N ""' % rsa_key_file_name)

# Set permissions of sshd chroot dir
os.system(r"chmod go-w /var/chroot/sshd")

# Download bootstrap-pkgsrc
pkgsrc_url_prefix = \
  r"http://segomo.elka.pw.edu.pl/qnx/QNX/i386/6.5.0_head_20100424/"
pkgsrc_our_url_prefix=r"http://segomo.elka.pw.edu.pl/qnx/QNX/i386/6.5.0_head_our_packages/"

bootstrap_file_url = pkgsrc_our_url_prefix + "6.5.0_bootstrap.tar.gz"

info("Downloading %s" % bootstrap_file_url)
bootstrap_file = urllib.urlretrieve(bootstrap_file_url, "/tmp/" + bname(bootstrap_file_url), urlretrieve_hook)[0]
info("")
info("Unpacking %s " % bname(bootstrap_file_url))
bootstrap_unpack_dir = r"/"
os.system("tar zxf \"%s\" -C \"%s\"" % (bootstrap_file, bootstrap_unpack_dir))

# Create pkgsrc temporary dir
pkgsrc_tmp_dir = r"/var/tmp"
if os.path.exists(pkgsrc_tmp_dir):
    warn(pkgsrc_tmp_dir + " already exists")
else:
    info("Creating " + pkgsrc_tmp_dir)
    os.system("mkdir \"%s\"" % pkgsrc_tmp_dir)

# Add pkgsrc related entries to /etc/profile
profile_file_name = r"/etc/profile"
add_cfg_file_entry(profile_file_name, \
  "PATH for pkgsrc",
  r"if test `id -u` == 0; then DUMMY_VAR=; PATH=/usr/pkg/bin:/usr/pkg/sbin:$PATH; else PATH=/usr/pkg/bin:$PATH; fi",
  r".*?DUMMY_VAR")

# Add required packages
packages = \
    "boost-headers-1.42.0", \
    "vim-share-7.2.411", \
    "vim-7.2.411", \
    "subversion-base-1.6.9nb1", \
    "scmgit-base-1.6.6.2", \
    "gsl-1.14", \
    "ncurses-5.7nb3", \
    "bash-4.1", \
    "bash-completion-1.0nb1"

for package in packages:
    os.system(r"/usr/pkg/sbin/pkg_add " + pkgsrc_url_prefix + "All/" + package + ".tgz")

our_packages = \
    "boost-libs-1.42.0", \
    "libxml2-2.7.7", \
    "xerces-c-3.0.1"

#    "cmake-2.8.0", \

for package in our_packages:
    os.system(r"/usr/pkg/sbin/pkg_add " + pkgsrc_our_url_prefix + "All/" + package + ".tgz")

# Add MRROC++ libraries
mrlib_svn_url=r"http://segomo.elka.pw.edu.pl/svn/mrrocpp/mrlib/"
mrlib_target_dir=os.environ["QNX_TARGET"] + "/mrlib"
if os.path.exists(mrlib_target_dir):
    info("Updating mrlib")
    os.system("svn update \"%s\"" % (mrlib_target_dir))
else:
    info("Checking out mrlib")
    os.system("svn checkout \"%s\" \"%s\"" % (mrlib_svn_url, mrlib_target_dir))

# Disable storing passwords in svn
enable_cfg_file_entry(r"/root/.subversion/config", "store_passwords=no", "store-passwords\s*=\s*no")

# This is no longer valid since we moved from SVN to Git
## Optional: checkout MRROC++
#mrrocpp_svn_target_dir = r"/home/mrrocpp"
#info("Checkout MRROC++ trunk to %s? [y/n] n" % mrrocpp_svn_target_dir)
#answer = ""
#while True:
#    co_mrrocpp = raw_input()
#    if co_mrrocpp == "y" or co_mrrocpp == "n" or co_mrrocpp == "":
#        break
#if (co_mrrocpp == "y"):
#    mrrocpp_svn_username=""
#    while True:
#        info("Enter svn username:")
#        mrrocpp_svn_username = raw_input()
#        if mrrocpp_svn_username != "":
#            break
#    mrrocpp_svn_url = r"https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/"
#    os.system("/usr/pkg/bin/svn checkout --username %s \"%s\" \"%s\"" % \
#      (mrrocpp_svn_username, mrrocpp_svn_url, mrrocpp_svn_target_dir))

info("Configuration complete. Please reboot.")
