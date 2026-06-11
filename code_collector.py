#!/usr/bin/env python3
"""
代码文件收集工具 - 将目录下的代码文件整合成 Markdown 文档
"""
import os
import argparse
from pathlib import Path
from datetime import datetime

# 常见代码文件扩展名
CODE_EXTENSIONS = {
    '.py', '.js', '.jsx', '.ts', '.tsx', '.java', '.c', '.cpp', '.h', '.hpp',
    '.cs', '.go', '.rs', '.rb', '.php', '.swift', '.kt', '.scala', '.r',
    '.m', '.mm', '.sh', '.bash', '.zsh', '.ps1', '.sql', '.html', '.css',
    '.scss', '.sass', '.less', '.vue', '.svelte', '.xml', '.json', '.yaml',
    '.yml', '.toml', '.ini', '.cfg', '.conf', '.md', '.rst', '.txt',
    '.gradle', '.maven', '.makefile', '.dockerfile', '.proto'
}

# 默认排除的目录
DEFAULT_EXCLUDE_DIRS = {
    'node_modules', '.git', '.svn', '.hg', '__pycache__', '.pytest_cache',
    'venv', 'env', '.venv', 'dist', 'build', '.idea', '.vscode', 
    'target', 'bin', 'obj', '.next', '.nuxt', 'coverage', "third_party",
    "build", "install", "log", "thirdparty", "local_libs"
}

# 默认排除的文件
DEFAULT_EXCLUDE_FILES = {
    '.DS_Store', 'Thumbs.db', '.gitignore', '.gitkeep', 
    'package-lock.json', 'yarn.lock', 'poetry.lock',
    "code_collector.py", "README.md", "LICENSE", "CHANGELOG.md",
    "code_collector.md"
}


def should_include_file(file_path, exclude_extensions, exclude_dirs, exclude_files):
    """判断文件是否应该被包含"""
    # 检查文件扩展名
    if file_path.suffix.lower() not in CODE_EXTENSIONS:
        return False
    
    if file_path.suffix.lower() in exclude_extensions:
        return False
    
    # 检查文件名
    if file_path.name in exclude_files:
        return False
    
    # 检查路径中是否包含排除的目录
    for part in file_path.parts:
        if part in exclude_dirs:
            return False
    
    return True


def get_language_from_extension(ext):
    """根据文件扩展名返回 Markdown 代码块的语言标识"""
    language_map = {
        '.py': 'python',
        '.js': 'javascript',
        '.jsx': 'jsx',
        '.ts': 'typescript',
        '.tsx': 'tsx',
        '.java': 'java',
        '.c': 'c',
        '.cpp': 'cpp',
        '.h': 'c',
        '.hpp': 'cpp',
        '.cs': 'csharp',
        '.go': 'go',
        '.rs': 'rust',
        '.rb': 'ruby',
        '.php': 'php',
        '.swift': 'swift',
        '.kt': 'kotlin',
        '.scala': 'scala',
        '.r': 'r',
        '.sh': 'bash',
        '.bash': 'bash',
        '.zsh': 'zsh',
        '.ps1': 'powershell',
        '.sql': 'sql',
        '.html': 'html',
        '.css': 'css',
        '.scss': 'scss',
        '.sass': 'sass',
        '.vue': 'vue',
        '.xml': 'xml',
        '.json': 'json',
        '.yaml': 'yaml',
        '.yml': 'yaml',
        '.toml': 'toml',
        '.md': 'markdown',
        '.dockerfile': 'dockerfile',
    }
    return language_map.get(ext.lower(), '')


def collect_code_files(root_dir, exclude_extensions, exclude_dirs, exclude_files, max_file_size=1024*1024):
    """收集所有符合条件的代码文件"""
    root_path = Path(root_dir).resolve()
    files_data = []
    
    for file_path in root_path.rglob('*'):
        if not file_path.is_file():
            continue
        
        # 获取相对路径用于判断
        try:
            rel_path = file_path.relative_to(root_path)
        except ValueError:
            continue
        
        if not should_include_file(rel_path, exclude_extensions, exclude_dirs, exclude_files):
            continue
        
        # 检查文件大小
        try:
            if file_path.stat().st_size > max_file_size:
                print(f"跳过大文件: {rel_path} (超过 {max_file_size/1024/1024:.1f}MB)")
                continue
        except OSError:
            continue
        
        # 读取文件内容
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            files_data.append({
                'path': str(rel_path),
                'content': content,
                'extension': file_path.suffix
            })
        except (UnicodeDecodeError, PermissionError) as e:
            print(f"无法读取文件: {rel_path} ({e})")
            continue
    
    return sorted(files_data, key=lambda x: x['path'])


def generate_markdown(files_data, output_file, project_name=None):
    """生成 Markdown 文档"""
    with open(output_file, 'w', encoding='utf-8') as f:
        # 写入标题和元信息
        title = project_name or "代码库文档"
        f.write(f"# {title}\n\n")
        f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"文件总数: {len(files_data)}\n\n")
        f.write("---\n\n")
        
        # 写入目录
        f.write("## 目录结构\n\n")
        for file_info in files_data:
            f.write(f"- `{file_info['path']}`\n")
        f.write("\n---\n\n")
        
        # 写入每个文件的内容
        f.write("## 文件内容\n\n")
        for file_info in files_data:
            f.write(f"### 📄 {file_info['path']}\n\n")
            
            language = get_language_from_extension(file_info['extension'])
            f.write(f"```{language}\n")
            f.write(file_info['content'])
            if not file_info['content'].endswith('\n'):
                f.write('\n')
            f.write("```\n\n")
            f.write("---\n\n")


def main():
    parser = argparse.ArgumentParser(
        description='将目录下的代码文件整合成 Markdown 文档',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                    # 收集当前目录所有代码
  %(prog)s -d ./src                           # 收集 src 目录的代码
  %(prog)s -o docs.md                         # 输出到 docs.md
  %(prog)s -e .test.js .spec.ts               # 排除测试文件
  %(prog)s --exclude-dir tests __tests__      # 排除测试目录
  %(prog)s -n "我的项目"                       # 指定项目名称
        """
    )
    
    parser.add_argument('-d', '--directory', 
                        default='.',
                        help='要扫描的根目录 (默认: 当前目录)')
    
    parser.add_argument('-o', '--output',
                        default='code_collector.md',
                        help='输出的 Markdown 文件名 (默认: code_collector.md)')
    
    parser.add_argument('-e', '--exclude-ext',
                        nargs='+',
                        default=[],
                        help='要排除的文件扩展名 (例如: .test.js .spec.ts)')
    
    parser.add_argument('--exclude-dir',
                        nargs='+',
                        default=[],
                        help='要排除的额外目录')
    
    parser.add_argument('--exclude-file',
                        nargs='+',
                        default=[],
                        help='要排除的文件名')
    
    parser.add_argument('-n', '--name',
                        help='项目名称')
    
    parser.add_argument('--max-size',
                        type=int,
                        default=1,
                        help='单个文件最大大小(MB) (默认: 1MB)')
    
    args = parser.parse_args()
    
    # 合并排除列表
    exclude_dirs = DEFAULT_EXCLUDE_DIRS | set(args.exclude_dir)
    exclude_files = DEFAULT_EXCLUDE_FILES | set(args.exclude_file)
    exclude_extensions = set(ext if ext.startswith('.') else f'.{ext}' 
                            for ext in args.exclude_ext)
    
    print(f"正在扫描目录: {args.directory}")
    print(f"排除的目录: {', '.join(sorted(exclude_dirs))}")
    if exclude_extensions:
        print(f"排除的扩展名: {', '.join(sorted(exclude_extensions))}")
    
    # 收集文件
    files_data = collect_code_files(
        args.directory,
        exclude_extensions,
        exclude_dirs,
        exclude_files,
        args.max_size * 1024 * 1024
    )
    
    if not files_data:
        print("未找到任何代码文件!")
        return
    
    print(f"找到 {len(files_data)} 个文件")
    
    # 生成 Markdown
    generate_markdown(files_data, args.output, args.name)
    
    print(f"✅ 成功生成文档: {args.output}")
    
    # 显示文件大小
    output_size = os.path.getsize(args.output)
    print(f"文档大小: {output_size/1024:.1f} KB")


if __name__ == '__main__':
    main()