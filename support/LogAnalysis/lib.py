import paramiko
from stat import S_ISDIR
from tqdm import tqdm
import os
import time
import ipywidgets as widgets
from IPython.display import display, clear_output

sftp = None

def make_sftp_widget(hosts=["sftp.tudelft.nl"]):
    # credentials
    boxStyle = {'width': '220px'}
    hostname = widgets.Dropdown(options=hosts, value=hosts[0], description='Select Host:', layout=boxStyle)
    port = 22
    user = widgets.Text(value='', placeholder='Enter username', description='username:', disabled=False, continuous_update=False, layout=boxStyle)
    pw = widgets.Password(value='', placeholder='Enter password', description='pass:', disabled=False, continuous_update=False, layout=boxStyle)

    def go(x):
        global sftp, files
        with output:
            clear_output()
            sftp, _ = connect_sftp(hostname.value, port, user.value, pw.value)
            pw.unobserve_all()
            pw.value = "allwehearisradiogaga" # destroy password, so it wont stay in notebook state

    button = widgets.Button(description="go!")

    output = widgets.Output()
    display(widgets.HBox((hostname, user, pw, button)), output)
    button.on_click(go)

def connect_sftp(hostname, port, username, password):
    print(f"Contacting file server... ", end='', flush=True)
    transport = paramiko.Transport((hostname, port))
    transport.connect(username=username, password=password)
    sftp = paramiko.SFTPClient.from_transport(transport)
    print(f"success!")
    return sftp, transport

def sftp_tree(sftp, path, depth=0, current_depth=0, prefix=""):
    """
    Emulates the `tree -L` functionality for an SFTP client.

    :param sftp_client: An instance of `paramiko.SFTPClient`.
    :param path: The root directory to start the tree.
    :param max_depth: Maximum depth to traverse.
    :param current_depth: The current depth in the recursion.
    :param prefix: Prefix for displaying hierarchy.
    """
    if current_depth > depth:
        return

    try:
        entries = sftp.listdir_attr(path)
    except IOError as e:
        print(f"{prefix}[ERROR] Unable to access '{path}': {e}")
        return

    for entry in entries:
        is_dir = entry.st_mode & 0o40000  # Check if the entry is a directory
        entry_path = f"{path}/{entry.filename}"
        print(f"{prefix}{entry.filename}/" if is_dir else f"{prefix}{entry.filename}")

        if is_dir and current_depth < depth:
            sftp_tree(sftp, entry_path, depth, current_depth + 1, prefix + "    ")

def list_files_by_extension(path, extension=None, max_depth=None):
    if extension is not None:
        extension = [x.lower() for x in extension]
        print(f"Scanning locally for {extension} files ", end='', flush=True)
    else:
        print(f"Scanning locally for any files ", end='', flush=True)

    files = []

    def recurse(rpath, depth):
        print('.', end='', flush=True)
        try:
            for entry in os.scandir(rpath):
                if entry.is_dir():
                    if max_depth is None or depth <= max_depth:
                        recurse(entry.path, depth+1)
                elif entry.is_file() and (extension is None or entry.filename.lower().split(".")[-1] in extension):
                    files.append(os.path.relpath(entry.path, path))
        except PermissionError:
            print(f"Permission denied: {rpath}")

    recurse(path, 1)
    print(f"\nFound {len(files)} matching files")
    return files

def sftp_list_files_by_extension(sftp, path, extension=None):
    if extension is not None:
        extension = [x.lower() for x in extension]
        print(f"Scanning remote for {extension} files ", end='', flush=True)
    else:
        print(f"Scanning remote for any files ", end='', flush=True)

    files = []

    def recurse(rpath):
        print('.', end='', flush=True)
        for entry in sftp.listdir_attr(rpath):
            entry_path = f"{rpath}/{entry.filename}"
            if S_ISDIR(entry.st_mode):
                recurse(entry_path)
            elif extension is None or entry.filename.lower().split(".")[-1] in extension:
                files.append(os.path.relpath(entry_path, path))

    recurse(path)
    print(f" Found {len(files)} matching files")
    return files

def make_selection_widget(path, cb, extension=None, max_depth=None):
    if isinstance(extension, str):
        extension = [extension]

    files = list_files_by_extension(path, extension=extension, max_depth=max_depth)
    files.sort(reverse=True)

    # Create the SelectMultiple widget
    button = widgets.Button(description="Select file")
    selector = widgets.Select(
        options=files,
        value=None,
        description='Files',
        disabled=False,
        layout={'width': '1000px', 'height': '300px'}
    )
    button = widgets.Button(description="Select!")

    def go(b):
        with output:
            full_path = os.path.join(path, selector.value)
            cb(full_path)

    output = widgets.Output()
    display(widgets.HBox([selector, button]), output)
    button.on_click(go)

    return files


def make_download_widget(sftp, local_path, remote_path, extension=None, multiple=False, cb=None, delete_after=False):
    if isinstance(extension, str):
        extension = [extension]

    files = sftp_list_files_by_extension(sftp, remote_path, extension=extension)
    files.sort(reverse=True)

    # Create the SelectMultiple widget
    widget = widgets.SelectMultiple if multiple else widgets.Select
    remote_files = widget(
        options=files,
        value=[] if multiple else None,
        description='Files',
        disabled=False,
        layout={'width': '1000px', 'height': '300px'}
    )
    button = widgets.Button(description="Download file(s)")

    def go(b):
        with output:
            clear_output()
            all_files = remote_files.value if multiple else [remote_files.value]
            for remote_file in all_files:
                full_local_path = os.path.join(local_path, os.path.dirname(remote_file))
                full_remote_file = os.path.join(remote_path, remote_file)
                try:
                    local_file = cache_file_with_progress(sftp, full_remote_file, full_local_path)
                    if cb is not None:
                        cb(local_file)
                finally:
                    if delete_after:
                        print(f"clearing cached file {local_file}")
                        try:
                            os.remove(local_file)
                        except:
                            pass
                    else:
                        print(f"file saved as {local_file}")
            print("done downloading files.")

    output = widgets.Output()
    display(widgets.HBox([remote_files, button]), output)
    button.on_click(go)

    return files

def make_upload_widget(sftp, local_path, remote_path, extension=None, multiple=False):
    if isinstance(extension, str):
        extension = [extension]

    files = list_files_by_extension(local_path, extension=extension)
    files.sort()

    # Create the SelectMultiple widget
    widget = widgets.SelectMultiple if multiple else widgets.Select
    local_files = widget(
        options=files,
        value=[] if multiple else None,
        description='Files',
        disabled=False,
        layout={'width': '1000px', 'height': '300px'}
    )
    button = widgets.Button(description="go!")

    def go(b):
        with output:
            clear_output()
            all_files = local_files.value if multiple else [local_files.value]
            for local_file in all_files:
                full_local_file = os.path.join(local_path, local_file)
                full_remote_path = os.path.join(remote_path, os.path.dirname(local_file))
                upload_file_with_progress(sftp, full_local_file, full_remote_path)
            print("done uploading files.")

    output = widgets.Output()
    display(widgets.HBox([local_files, button]), output)
    button.on_click(go)

    return files

def progress_callback(bar, transferred, total):
    bar.update(transferred - bar.n)

def cache_file_with_progress(sftp, remote_file, local_path):
    os.makedirs(local_path, exist_ok=True)
    local_file = os.path.join(local_path, os.path.basename(remote_file))

    progress_bar = tqdm(total=sftp.stat(remote_file).st_size,
                        unit='B', unit_scale=True,
                        desc=os.path.basename(remote_file))
    sftp.get(remote_file, local_file,
             callback=lambda b, t: progress_callback(progress_bar, b, t))
    progress_bar.close()

    return local_file

def sftp_makedirs(sftp, remote_path):
    """
    Recursively create directories on the SFTP server.
    Equivalent to os.makedirs(remote_path, exist_ok=True) for remote paths.
    """
    dirs = []
    current_path = remote_path
    while current_path not in ("", "/"):
        dirs.append(current_path)
        current_path = os.path.dirname(current_path)
    dirs.reverse()  # Start creating from the top-most directory

    for dir_path in dirs:
        try:
            sftp.stat(dir_path)  # Check if directory exists
        except FileNotFoundError:
            sftp.mkdir(dir_path)  # Create directory if it doesn't exist

def upload_file_with_progress(sftp, local_file, remote_path):
    sftp_makedirs(sftp, remote_path)
    remote_file = os.path.join(remote_path, os.path.basename(local_file))

    progress_bar = tqdm(total=os.stat(local_file).st_size,
                        unit='B', unit_scale=True,
                        desc=os.path.basename(local_file))
    sftp.put(local_file, remote_file,
             callback=lambda b, t: progress_callback(progress_bar, b, t))
    progress_bar.close()

#def make_rerun_widget(local_file, width=1720, height=950):
#    print("starting rerun widget... ")
#    rr.init("")
#    rr.log_file_from_path(local_file)
#    rr.notebook_show(width=width, height=height)
#
#def spawn_rerun_or_add(local_file):
#    rr.init("Log Viewer")
#    rr.log_file_from_path(local_file)
#    rr.spawn()


ZMQ_PORT = 9872
SLEEP_TIME = 0.0005

def stream_df_over_zmq(df, port=ZMQ_PORT, sleep_time=SLEEP_TIME):
    import zmq
    import bson

    socket = zmq.Context().socket(zmq.PUB)
    socket.bind(f"tcp://*:{port}")
    try:
        for _, row in tqdm(df.iterrows(), total=len(df), desc="Streaming Log"):
            row_dict = row.to_dict()
            socket.send(bson.encode(row_dict))
            time.sleep(sleep_time) # limit network traffic to like 10MB/s instead of 50MB/s
    except KeyboardInterrupt:
        pass
    finally:
        socket.close()

#def make_indiflight_zmq_server(local_file, log_cb=None):
#    log = IndiflightLog(local_file, resetTime=True)
#    stream_df_over_zmq(log.data)
#
#    if log_cb is not None:
#        log_cb(log)
