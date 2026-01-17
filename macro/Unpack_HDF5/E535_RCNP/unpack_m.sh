#!/bin/bash

logDir="./logUnpack"
MAX_THREADS=2  # 最大线程数

# 显示用法
usage() {
    echo "Usage: $0 runNum1 [runNum2]"
    echo "       If only one run number is provided, process only that run."
    echo "       If two run numbers are provided, process all runs between them (inclusive)."
    echo "Example:"
    echo "  $0 1000          # Process only run 1000"
    echo "  $0 1000 1005     # Process runs 1000, 1001, 1002, 1003, 1004, 1005"
    exit 1
}

# 检查参数
if [ $# -lt 1 ]; then
    usage
fi

# 获取开始和结束run号
if [ $# -eq 1 ]; then
    # 只有一个参数，只处理这个run
    startRun=$1
    endRun=$1
elif [ $# -eq 2 ]; then
    # 两个参数，处理范围内的所有run
    startRun=$1
    endRun=$2
    
    # 确保startRun <= endRun
    if [ $startRun -gt $endRun ]; then
        # 交换顺序
        temp=$startRun
        startRun=$endRun
        endRun=$temp
        echo "Note: Swapped order to $startRun -> $endRun"
    fi
else
    usage
fi

# 计算总run数
totalRuns=$((endRun - startRun + 1))
echo "========================================"
echo "Processing runs: $startRun to $endRun"
echo "Total runs: $totalRuns"
echo "Max concurrent threads: $MAX_THREADS"
echo "Log directory: $logDir"
echo "========================================"

# 创建日志目录
mkdir -p "$logDir"

# 多线程处理函数
process_runs_parallel() {
    local start=$1
    local end=$2
    
    # 计数器
    local current=$start
    local completed=0
    
    # 存储进程ID的数组
    declare -A pids
    
    # 初始化进程数组
    for ((i=0; i<MAX_THREADS; i++)); do
        pids[$i]=0
    done
    
    echo "Starting parallel processing..."
    
    # 主循环
    while [ $completed -lt $totalRuns ]; do
        # 检查是否有空闲的线程槽
        for ((slot=0; slot<MAX_THREADS; slot++)); do
            # 如果这个slot有进程，检查是否完成
            if [ ${pids[$slot]} -ne 0 ]; then
                if ! kill -0 ${pids[$slot]} 2>/dev/null; then
                    # 进程已完成
                    wait ${pids[$slot]}
                    exit_code=$?
                    if [ $exit_code -eq 0 ]; then
                        echo "✓ Run ${run_nums[$slot]} completed successfully"
                    else
                        echo "✗ Run ${run_nums[$slot]} failed with exit code $exit_code"
                    fi
                    pids[$slot]=0
                    completed=$((completed + 1))
                    
                    # 显示进度
                    progress=$((completed * 100 / totalRuns))
                    echo "[$completed/$totalRuns] Progress: $progress%"
                fi
            fi
            
            # 如果有空闲slot并且还有run要处理
            if [ ${pids[$slot]} -eq 0 ] && [ $current -le $end ]; then
                runNum=$current
                logFile="$logDir/run${runNum}.log"
                
                echo "→ Starting run $runNum (slot $slot)"
                
                # 启动处理进程
                (
                    # 设置进程标题以便识别
                    printf "\033]0;Unpack Run $runNum\007"
                    
                    # 运行ROOT解包程序
                    root -b -q -l "unpack_rcnp.C($runNum)" > "$logFile" 2>&1
                    
                    exit_code=$?
                    if [ $exit_code -eq 0 ]; then
                        echo "  Run $runNum: Success" >> "$logDir/summary.log"
                    else
                        echo "  Run $runNum: FAILED (exit code: $exit_code)" >> "$logDir/summary.log"
                    fi
                    exit $exit_code
                ) &
                
                # 存储进程ID和对应的run号
                pids[$slot]=$!
                run_nums[$slot]=$runNum
                
                # 更新当前run号
                current=$((current + 1))
                
                # 短暂延迟避免同时启动太多进程
                sleep 0.5
            fi
        done
        
        # 如果没有空闲slot，等待一会儿再检查
        if [ $completed -lt $totalRuns ]; then
            sleep 1
        fi
    done
    
    echo "All runs completed!"
}

# 单线程处理函数（备用）
process_runs_sequential() {
    local start=$1
    local end=$2
    
    echo "Starting sequential processing..."
    
    for ((runNum=start; runNum<=end; runNum++)); do
        logFile="$logDir/run${runNum}.log"
        current=$((runNum - start + 1))
        
        echo "[$current/$totalRuns] Processing run: $runNum"
        
        # 运行ROOT解包程序
        root -b -q -l "unpack_rcnp.C($runNum)" > "$logFile" 2>&1
        
        if [ $? -eq 0 ]; then
            echo "  ✓ Run $runNum completed"
            echo "  Run $runNum: Success" >> "$logDir/summary.log"
        else
            echo "  ✗ Run $runNum failed"
            echo "  Run $runNum: FAILED" >> "$logDir/summary.log"
        fi
    done
}

# 显示选项菜单
echo ""
echo "Select processing mode:"
echo "  1) Parallel processing (up to $MAX_THREADS threads)"
echo "  2) Sequential processing (one by one)"
echo "  3) Dry run (show what would be processed)"
read -p "Enter choice [1]: " choice

choice=${choice:-1}  # 默认选择1

case $choice in
    1)
        # 清空之前的summary日志
        > "$logDir/summary.log"
        echo "Parallel processing started at: $(date)" >> "$logDir/summary.log"
        
        # 开始时间
        start_time=$(date +%s)
        
        # 并行处理
        process_runs_parallel $startRun $endRun
        
        # 结束时间
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        echo "Total processing time: $duration seconds" >> "$logDir/summary.log"
        
        # 显示summary
        echo ""
        echo "========================================"
        echo "PROCESSING SUMMARY"
        echo "========================================"
        cat "$logDir/summary.log"
        ;;
        
    2)
        # 清空之前的summary日志
        > "$logDir/summary.log"
        echo "Sequential processing started at: $(date)" >> "$logDir/summary.log"
        
        # 开始时间
        start_time=$(date +%s)
        
        # 顺序处理
        process_runs_sequential $startRun $endRun
        
        # 结束时间
        end_time=$(date +%s)
        duration=$((end_time - start_time))
        echo "Total processing time: $duration seconds" >> "$logDir/summary.log"
        
        # 显示summary
        echo ""
        echo "========================================"
        echo "PROCESSING SUMMARY"
        echo "========================================"
        cat "$logDir/summary.log"
        ;;
        
    3)
        # Dry run模式
        echo ""
        echo "========================================"
        echo "DRY RUN - Runs to be processed:"
        echo "========================================"
        for ((runNum=startRun; runNum<=endRun; runNum++)); do
            echo "  Run $runNum"
        done
        echo ""
        echo "Log files would be saved to: $logDir/"
        echo "Total runs: $totalRuns"
        echo "Parallel mode would use up to $MAX_THREADS threads"
        exit 0
        ;;
        
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo ""
echo "\(>.<)/ Unpacking is completed (>=<)"
echo "Log files are in: $logDir/"
echo "Summary: $logDir/summary.log"
