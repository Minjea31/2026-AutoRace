"""Small, read-only host/domain discovery shared by launcher, logs and web UI."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def domain_number(value):
    try:number=int(value)
    except (ValueError,TypeError):raise ValueError('ROS_DOMAIN_ID는 0~232 사이 정수여야 합니다.')
    if not 0<=number<=232:raise ValueError('ROS_DOMAIN_ID는 0~232 사이 정수여야 합니다.')
    return number


def detect_domains(proc=Path('/proc')):
    """Inspect same-user ROS process environments only. Never return other env data."""
    groups={}
    for directory in proc.iterdir():
        if not directory.name.isdigit() or int(directory.name)==os.getpid():continue
        try:
            if directory.stat().st_uid!=os.getuid():continue
            raw=(directory/'cmdline').read_bytes().split(b'\0')
            args=[p.decode(errors='replace') for p in raw if p]
            if any('web_viewer' in p or 'mission_runtime.py' in p or 'mission_web.launch.py' in p or 'ros2-daemon' in p or 'ros2cli.daemon' in p for p in args):continue
            # Match executable/path arguments, not shell snippets (which may mention ROS).
            paths=[p for p in args if '\n' not in p and ' ' not in p]
            relevant=any('/opt/ros/' in p or '/2026-AutoRace/' in p or '_ws/' in p or Path(p).name in ('ros2','component_container','robot_state_publisher') for p in paths)
            if not relevant:continue
            env=dict(item.split(b'=',1) for item in (directory/'environ').read_bytes().split(b'\0') if b'=' in item)
            domain=domain_number(env.get(b'ROS_DOMAIN_ID',b'0'))
            name=next((Path(p).name for p in paths if p.endswith('.py')),Path(paths[0]).name if paths else 'ROS')
            group=groups.setdefault(domain,dict(domain=domain,process_count=0,processes=set()))
            group['process_count']+=1;group['processes'].add(name)
        except (OSError,PermissionError,ValueError):continue
    return [dict(v,processes=sorted(v['processes'])) for _,v in sorted(groups.items())]


def select_domain(environ=None,detected=None):
    environ=os.environ if environ is None else environ
    if environ.get('ROS_DOMAIN_ID') not in (None,''):
        return domain_number(environ['ROS_DOMAIN_ID']),'environment'
    detected=detect_domains() if detected is None else detected
    domains={r['domain'] for r in detected}
    if len(domains)==1:return domains.pop(),'running_ros_processes'
    return 99,'default'


def network_addresses(port,host='0.0.0.0'):
    addresses=[]
    try:
        result=subprocess.run(['ip','-j','-4','addr','show','scope','global'],capture_output=True,text=True,timeout=1,check=True)
        for interface in json.loads(result.stdout):
            if interface.get('operstate')!='UP':continue
            for addr in interface.get('addr_info',[]):
                ip=addr['local']
                if host not in ('0.0.0.0',ip):continue
                addresses.append(dict(interface=interface['ifname'],ip=ip,url=f'http://{ip}:{port}'))
    except (OSError,ValueError,subprocess.SubprocessError):pass
    if host.startswith('127.'):
        addresses=[dict(interface='lo',ip=host,url=f'http://{host}:{port}')]
    return addresses


class RuntimeMonitor:
    def __init__(self,state,domain,port,host='0.0.0.0',source='environment',scanner=detect_domains,address_reader=network_addresses,clock=time.monotonic):
        self.state=state;self.domain=domain;self.port=port;self.host=host;self.source=source
        self.scanner=scanner;self.address_reader=address_reader;self.clock=clock
        self.last_scan=-1e12;self.last_network=-1e12;self.current={};self.signature=None
    def poll(self,force=False):
        now=self.clock()
        if not force and now-self.last_scan<5:return
        self.last_scan=now
        observed=self.scanner();domains=[v['domain'] for v in observed]
        addresses=self.current.get('addresses',[])
        if force or now-self.last_network>=15:
            addresses=self.address_reader(self.port,self.host);self.last_network=now
        mismatch=bool(domains) and any(v!=self.domain for v in domains)
        if mismatch:
            warning=f'도메인 불일치: 웹 ROS_DOMAIN_ID={self.domain}, 로봇 ROS 프로세스={domains}. 도메인을 맞춰 웹을 다시 실행하세요.'
        else:warning=''
        signature=(tuple(domains),tuple(a['url'] for a in addresses))
        previous=self.current.get('observed_domains')
        self.current=dict(domain_id=self.domain,domain_source=self.source,observed_domains=domains,observed_processes=observed,
                          domain_mismatch=mismatch,warning=warning,addresses=addresses,localhost_only=os.environ.get('ROS_LOCALHOST_ONLY','0'),check_interval_s=5)
        if signature!=self.signature:
            change=f'ROS 프로세스 도메인 변경: {previous} → {domains}. ' if previous is not None and previous!=domains else ''
            message=change+f'웹 ROS_DOMAIN_ID={self.domain}; 로봇 ROS 프로세스={domains or "미감지"}; 접속='+(', '.join(a['url'] for a in addresses) or '활성 LAN 주소 없음')
            if warning:message+='; '+warning
            print('[runtime] '+message,flush=True)
            self.state.set('runtime',message,True)
            self.signature=signature
    def snapshot(self):return dict(self.current)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--select-domain',action='store_true');parser.add_argument('--domain-source',action='store_true');parser.add_argument('--select-config',action='store_true');args=parser.parse_args()
    try:domain,source=select_domain()
    except ValueError as error:parser.error(str(error))
    print(f'{domain} {source}' if args.select_config else source if args.domain_source else domain)
if __name__=='__main__':main()
