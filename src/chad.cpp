#include "chad.h"

namespace chad {

std::shared_ptr<Function> g_Function;

ExpressionCPtr Forward(const ExpressionCPtr &expr,
                       const std::unordered_map<ExpressionCPtr, int> &wrtMap,
                       const ExpressionCPtrVec &dDir,
                       std::unordered_map<ExpressionCPtr, ExpressionCPtr> &fwdExprs) {
    if (fwdExprs.find(expr) != fwdExprs.end()) {
        return fwdExprs[expr];
    }

    ExpressionCPtr ret;
    auto wrtMapIt = wrtMap.find(expr);
    if (wrtMapIt != wrtMap.end()) {
        assert(wrtMapIt->second < int(dDir.size()));
        ret = dDir[wrtMapIt->second];
    } else {
        ret = Constant::Create(0.0);
        auto children = expr->Children();
        auto dervs = expr->Dervs();
        for (int childId = 0; childId < (int)children.size(); childId++) {
            auto fwd = Forward(children[childId], wrtMap, dDir, fwdExprs);
            ret += fwd * dervs[childId];
        }
    }
    fwdExprs[expr] = ret;
    return ret;
}

void Forward(const ExpressionCPtrVec &exprs,
             const ExpressionCPtrVec &wrt,
             const ExpressionCPtrVec &dDir,
             std::unordered_map<ExpressionCPtr, ExpressionCPtr> &fwdExprs) {
    std::unordered_map<ExpressionCPtr, int> wrtMap;
    int wrtId = 0;
    for (auto &expr : wrt) {
        wrtMap[expr] = wrtId;
        wrtId++;
    }
    for (auto &expr : exprs) {
        Forward(expr, wrtMap, dDir, fwdExprs);
    }
}

void Emit(EmitHelper &helper, const ExpressionCPtrVec &exprs, std::ostream &os) {
    for (auto &expr : exprs) {
        expr->Register(helper);
    }
    for (auto &expr : exprs) {
        expr->EmitIfNotEmitted(helper, os);
    }
}

void Forward(std::shared_ptr<CFGBlock> &block,
             const ExpressionCPtrVec &wrt,
             const ExpressionCPtrVec &dExprVec,
             std::unordered_map<ExpressionCPtr, ExpressionCPtr> &fwdExprs) 
{
    g_Function->curBlock = std::make_shared<CFGBlock>();
    Forward(block->exprs, wrt, dExprVec, fwdExprs);
    block->exprs.insert(
        block->exprs.end(), g_Function->curBlock->exprs.begin(), g_Function->curBlock->exprs.end());
    g_Function->curBlock = nullptr;
    if (block->next.get() != nullptr) {
        auto split = block->next;
        int childId = 0;
        std::vector<std::vector<ExpressionCPtr>> possibleExprs(split->outputs.size());
        for (auto child : split->children) {
            Forward(child, wrt, dExprVec, fwdExprs);
            for (int oid = 0; oid < (int)split->outputs.size(); oid++) {
                auto expr = split->outputs[oid]->GetPossibleExpr(childId);
                if (fwdExprs.find(expr) == fwdExprs.end()) {
                    possibleExprs[oid].push_back(Constant::Create(0.0));
                } else {
                    possibleExprs[oid].push_back(fwdExprs[expr]);
                }
            }
            childId++;
        }
        std::vector<CondExprCPtr> newOutputs;
        for (int oid = 0; oid < (int)split->outputs.size(); oid++) {
            auto condExpr = CondExpr::Create();
            for (auto expr : possibleExprs[oid]) {
                condExpr->AddPossibleExpr(expr);
            }
            newOutputs.push_back(condExpr);
            fwdExprs[split->outputs[oid]] = condExpr;
        }
        split->outputs.insert(split->outputs.end(), newOutputs.begin(), newOutputs.end());

        if (split->next.get() != nullptr) {
            Forward(split->next, wrt, dExprVec, fwdExprs);
        }
    }
}

void Forward(const std::shared_ptr<Function> &func,
             const ExpressionCPtrVec &wrt,
             std::unordered_map<ExpressionCPtr, ExpressionCPtr> &fwdExprs) {
    auto dArg = Argument("d", (int)wrt.size());
    auto dArgVec = dArg.GetExprVec();
    func->firstBlock->exprs.insert(func->firstBlock->exprs.begin(), dArgVec.begin(), dArgVec.end());
    Forward(func->firstBlock, wrt, dArgVec, fwdExprs);
}

void Emit(EmitHelper &helper,
          const std::shared_ptr<CFGBlock> &block,
          std::ostream &os,
          const bool ispcMode = false) 
{
    Emit(helper, block->exprs, os);
    if (block->next.get() != nullptr) {
        auto split = block->next;
        int childId = 0;
        for (auto output : split->outputs) {
            helper.Register(output);
        }
        for (auto cond : split->conditions) {
            if (cond.get() != nullptr) {
                cond->Register(helper);
            }
        }

        for (auto child : split->children) {
            if (childId == 0) {
                split->conditions[childId]->EmitAll(helper, os);
                helper.PrintTab(os);
                if (ispcMode) {
                    os << "cif (";
                } else {
                    os << "if (";
                }
                os << split->conditions[childId]->GetEmitName(helper) << ") {" << std::endl;
            } else if (childId != int(split->children.size()) - 1) {
                split->conditions[childId]->EmitAll(helper, os);
                helper.PrintTab(os);
                if (ispcMode) {
                    os << "else cif (";
                } else {
                    os << "else if (";
                }
                os << split->conditions[childId]->GetEmitName(helper) << ") {" << std::endl;
            } else {
                helper.PrintTab(os);
                os << "else {" << std::endl;
            }
            helper.IncTab();
            Emit(helper, child, os, ispcMode);
            for (auto output : split->outputs) {
                auto expr = output->GetPossibleExpr(childId);
                helper.PrintTab(os);
                os << output->GetEmitName(helper) << " = " << expr->GetEmitName(helper) << ";"
                   << std::endl;
            }
            helper.DecTab();
            helper.PrintTab(os);
            os << "}" << std::endl;
            childId++;
        }
        for (auto output : split->outputs) {
            helper.SetEmitted(output);
        }
        if (split->next.get() != nullptr) {
            Emit(helper, split->next, os, ispcMode);
        }
    }
}

void Emit(const std::shared_ptr<Function> &func, std::ostream &os) 
{
    EmitHelper helper;
    os << "#include <math.h>" << std::endl;
    os << "void " << func->name << "(";
    bool first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        os << "const " << input.GetDeclaration();
        first = false;
    }
    for (auto &output : func->outputs) {
        if (!first) {
            os << ", ";
        }
        os << c_FloatTypeDecl << output.first << "[" << output.second.size() << "]";
    }
    os << ") {" << std::endl;
    helper.IncTab();
    std::stringstream ss;
    Emit(helper, func->firstBlock, ss);

    auto emitted = helper.GetEmitted();
    if (emitted.size() > 0) {
        helper.PrintTab(os);
        os << c_FloatTypeDecl;
        bool first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (!first) {
                os << ", ";
            }
            os << expr->GetEmitName(helper);
            first = false;
        }
        os << ";" << std::endl;
    }
    os << ss.str();

    for (auto &output : func->outputs) {
        int subOutputId = 0;
        for (auto &subOutput : output.second) {
            helper.PrintTab(os);
            os << output.first << "[" << subOutputId << "] = " << subOutput->GetEmitName(helper)
               << ";" << std::endl;
            subOutputId++;
        }
    }
    helper.DecTab();
    os << "}" << std::endl;
}

void EmitReverse(EmitHelper &helper,
                 const std::shared_ptr<CFGBlock> &block,
                 std::unordered_set<ExpressionCPtr> &nonZeroAccId,
                 std::ostream &os,
                 const bool ispcMode = false) 
{
    if (block->next.get() != nullptr) {
        auto split = block->next;
        if (split->next.get() != nullptr) {
            EmitReverse(helper, split->next, nonZeroAccId, os, ispcMode);
        }
        int childId = 0;
        for (auto child : split->children) {
            bool hasNonZeroAcc = false;
            for (auto output : split->outputs) {
                if (nonZeroAccId.find(output) != nonZeroAccId.end()) {
                    hasNonZeroAcc = true;
                    break;
                }
            }
            if (!hasNonZeroAcc) {
                continue;
            }
            if (childId == 0) {
                split->conditions[childId]->EmitAll(helper, os);
                helper.PrintTab(os);
                if (ispcMode) {
                    os << "cif (";
                } else {
                    os << "if (";
                }
                os << split->conditions[childId]->GetEmitName(helper) << ") {" << std::endl;
            } else if (childId != int(split->children.size()) - 1) {
                split->conditions[childId]->EmitAll(helper, os);
                helper.PrintTab(os);
                if (ispcMode) {
                    os << "else cif (";
                } else {
                    os << "else if (";
                }
                os << split->conditions[childId]->GetEmitName(helper) << ") {" << std::endl;
            } else {
                helper.PrintTab(os);
                os << "else {" << std::endl;
            }
            helper.IncTab();
            for (auto output : split->outputs) {
                if (nonZeroAccId.find(output) == nonZeroAccId.end()) {
                    continue;
                }
                auto expr = output->GetPossibleExpr(childId);
                if (!helper.ExprRegistered(expr)) {
                    continue;
                }
                helper.PrintTab(os);
                os << "_acc" << helper.GetExprId(expr) << " = "
                   << "_acc" << helper.GetExprId(output) << ";" << std::endl;
                nonZeroAccId.insert(expr);
            }
            EmitReverse(helper, child, nonZeroAccId, os, ispcMode);
            helper.DecTab();
            helper.PrintTab(os);
            os << "}" << std::endl;
            childId++;
        }
    }

    for (auto it = block->exprs.rbegin(); it != block->exprs.rend(); it++) {
        auto expr = *it;
        if (nonZeroAccId.find(expr) == nonZeroAccId.end()) {
            continue;
        }
        auto children = expr->Children();
        auto dervs = expr->Dervs();
        for (int childId = 0; childId < (int)children.size(); childId++) {
            auto child = children[childId];
            if (!helper.ExprRegistered(child)) {
                continue;
            }
            auto derv = dervs[childId];
            if (derv->IsConstant() && derv->GetConstantVal() == 1.0) {
                helper.PrintTab(os);
                os << "_acc" << helper.GetExprId(child) << " += "
                   << "_acc" << helper.GetExprId(expr) << ";" << std::endl;
                nonZeroAccId.insert(child);
            } else if (derv->IsConstant() && derv->GetConstantVal() == -1.0) {
                helper.PrintTab(os);
                os << "_acc" << helper.GetExprId(child) << " -= "
                   << "_acc" << helper.GetExprId(expr) << ";" << std::endl;
                nonZeroAccId.insert(child);
            } else if (!derv->IsConstant() || derv->GetConstantVal() != 0.0) {
                derv->Register(helper);
                derv->EmitAll(helper, os);
                helper.PrintTab(os);
                os << "_acc" << helper.GetExprId(child) << " += "
                   << "_acc" << helper.GetExprId(expr) << " * " << derv->GetEmitName(helper) << ";"
                   << std::endl;
                nonZeroAccId.insert(child);
            }
        }
    }
}

void EmitGradHessian(const std::shared_ptr<Function> &func,
                     const ExpressionCPtrVec &wrt,
                     const ExpressionCPtr &dep,
                     std::ostream &os,
                     const bool emitIspc) 
{
    EmitHelper helper;
    if (!emitIspc) {
        os << "#include <math.h>" << std::endl;
    }
    os << "static void " << GetDervName(func->name) << "_kernel(";
    bool first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false;
    }

    if (wrt.size() > 0) {
        if (!first) {
            os << ", ";
        }
        os << "const " << c_FloatTypeDecl << "d[" << wrt.size() << "]";
        os << ", ";
        os << c_FloatTypeDecl << "grad[" << 1 << "]";
        os << ", ";
        os << c_FloatTypeDecl << "hess[" << wrt.size() << "]";
    }

    os << ") {" << std::endl;

    std::stringstream forwardStream;
    helper.IncTab();
    helper.PrintTab(forwardStream);
    forwardStream << "/* Forward */" << std::endl;

    for (auto &expr : wrt) {
        assert(expr.get() != nullptr);
        helper.Register(expr);
    }

    std::unordered_map<ExpressionCPtr, ExpressionCPtr> fwdExprs;
    Forward(func, wrt, fwdExprs);

    for (auto &expr : wrt) {
        helper.Register(fwdExprs[expr]);
    }
    Emit(helper, func->firstBlock, forwardStream, emitIspc);

    assert(dep.get() != nullptr);
    auto fDep = fwdExprs[dep];
    assert(fDep.get() != nullptr);
    std::unordered_set<ExpressionCPtr> nonZeroAccId;
    std::stringstream backwardStream;
    if (helper.ExprRegistered(fDep)) {
        helper.PrintTab(forwardStream);
        forwardStream << "grad[0] = " << fDep->GetEmitName(helper) << ";" << std::endl;

        helper.PrintTab(backwardStream);
        backwardStream << "/* Reverse accumulation */" << std::endl;

        helper.PrintTab(backwardStream);
        backwardStream << "_acc" << helper.GetExprId(fDep) << " = 1.0;" << std::endl;
        nonZeroAccId.insert(fDep);
        EmitReverse(helper, func->firstBlock, nonZeroAccId, backwardStream, emitIspc);
        int id = 0;
        for (auto &expr : wrt) {
            helper.PrintTab(backwardStream);
            backwardStream << "hess[" << id << "] = _acc" << helper.GetExprId(expr) << ";"
                           << std::endl;
            id++;
        }
    }

    std::stringstream backwardDeclStream;
    auto emitted = helper.GetEmitted();
    if (emitted.size() > 0) {
        helper.PrintTab(os);
        if (emitIspc) {
            os << "uniform ";
        }
        os << c_FloatTypeDecl;
        bool first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (fwdExprs.find(expr) != fwdExprs.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;
        helper.PrintTab(os);
        os << c_FloatTypeDecl;
        first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (fwdExprs.find(expr) == fwdExprs.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;
        helper.PrintTab(backwardDeclStream);
        backwardDeclStream << c_FloatTypeDecl;
        first = true;
        for (auto &expr : wrt) {
            if (nonZeroAccId.find(expr) != nonZeroAccId.end()) {
                continue;
            }
            if (!first) {
                backwardDeclStream << ", ";
            }
            backwardDeclStream << "_acc" << helper.GetExprId(expr) << " = 0";
            first = false;
        }
        for (auto &expr : nonZeroAccId) {
            if (!first) {
                backwardDeclStream << ", ";
            }
            backwardDeclStream << "_acc" << helper.GetExprId(expr) << " = 0";
            first = false;
        }
        backwardDeclStream << ";" << std::endl;
    }

    os << forwardStream.str();
    os << backwardDeclStream.str();
    os << backwardStream.str();
    helper.DecTab();
    os << "}" << std::endl;

    if (emitIspc) {
        os << "export ";
    }
    os << "void " << GetDervName(func->name) << "(";
    first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false;
    }
    os << ", ";
    if (emitIspc) {
        os << "uniform ";
    }
    os << c_FloatTypeDecl << "grad[]";
    os << ", ";
    if (emitIspc) {
        os << "uniform ";
    }
    os << c_FloatTypeDecl << "hess[]";
    os << ") {" << std::endl;
    int dim = (int)wrt.size();
    if (emitIspc) {
        os << "\tforeach (index = 0 ... " << dim << ") {" << std::endl;
    } else {
        os << "\tfor (int index = 0; index < " << dim << "; index++) {" << std::endl;
    }
    os << "\t\t" << c_FloatTypeDecl << "d[" << dim << "] = {0};" << std::endl;
    os << "\t\t" << c_FloatTypeDecl << "g;" << std::endl;
    os << "\t\t" << c_FloatTypeDecl << "h[" << dim << "];" << std::endl;
    os << "\t\t"
       << "d[index] = 1;" << std::endl;
    os << "\t\t" << GetDervName(func->name) << "_kernel(";
    first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        os << input.GetName();
        first = false;
    }
    os << ", d, &g, h);" << std::endl;
    os << "\t\t"
       << "grad[index] = g;" << std::endl;
    if (emitIspc) {
        os << "\t\t"
           << "for (uniform int i = 0; i < " << dim << "; i++) {" << std::endl;
    } else {
        os << "\t\t"
           << "for (int i = 0; i < " << dim << "; i++) {" << std::endl;
    }
    os << "\t\t\t"
       << "hess[index * " << dim << " + i] = h[i];" << std::endl;
    os << "\t\t}" << std::endl;
    os << "\t}" << std::endl;
    os << "}" << std::endl;
}

// Forward mode autodiff
void EmitGrad(const std::shared_ptr<Function> &func,
              const ExpressionCPtrVec &wrt, 
              const ExpressionCPtr &dep, 
              std::ostream &os, 
              const bool emitIspc)
{
    EmitHelper helper;
    if (!emitIspc) {
        os << "#include <math.h>" << std::endl;
    }
    os << "static void " << GetDervName(func->name) << "_kernel(";
    bool first = true; 
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false; 
    }

    if (wrt.size() > 0) {
        if (!first) {
            os << ", ";
        }
        os << "const " << c_FloatTypeDecl << "d[" << wrt.size() << "]";
        os << ", ";
        os << c_FloatTypeDecl << "grad[" << 1 << "]";
    }

    os << ") {" << std::endl; 

    std::stringstream forwardStream; 
    helper.IncTab(); 
    helper.PrintTab(forwardStream);
    forwardStream << "/* Forward */" << std::endl;

    for (auto &expr : wrt) {
        assert(expr.get() != nullptr);
        helper.Register(expr);
    }

    std::unordered_map<ExpressionCPtr, ExpressionCPtr> fwdExprs;
    Forward(func, wrt, fwdExprs);

    for (auto &expr : wrt) {
        helper.Register(fwdExprs[expr]);
    }
    Emit(helper, func->firstBlock, forwardStream, emitIspc);

    assert(dep.get() != nullptr);
    auto fDep = fwdExprs[dep];
    assert(fDep.get() != nullptr);
    std::unordered_set<ExpressionCPtr> nonZeroAccId;
    if (helper.ExprRegistered(fDep)) {
        helper.PrintTab(forwardStream);
        forwardStream << "grad[0] = " << fDep->GetEmitName(helper) << ";" << std::endl;
    }

    auto emitted = helper.GetEmitted();
    if (emitted.size() > 0) {
        helper.PrintTab(os);
        if (emitIspc) {
            os << "uniform ";
        }
        os << c_FloatTypeDecl;
        bool first = true; 
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (fwdExprs.find(expr) != fwdExprs.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;
        helper.PrintTab(os);
        os << c_FloatTypeDecl;
        first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (fwdExprs.find(expr) == fwdExprs.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;
    }

    os << forwardStream.str();
    helper.DecTab();
    os << "}" << std::endl;

    if (emitIspc) {
        os << "export ";
    }
    os << "void " << GetDervName(func->name) << "(";
    first = true; 
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false; 
    }
    os << ", ";
    if (emitIspc) {
        os << "uniform ";
    }
    os << c_FloatTypeDecl << "grad[]"; 
    os << ") {" << std::endl;
    int dim = (int)wrt.size(); 
    if (emitIspc) {
        os << "\tforeach (index = 0 ... " << dim << ") {" << std::endl;
    } else {
        os << "\tfor (int index = 0; index < " << dim << "; index++) {" << std::endl;
    }
    os << "\t\t" << c_FloatTypeDecl << "d[" << dim << "] = {0};" << std::endl;
    os << "\t\t" << c_FloatTypeDecl << "g;" << std::endl;
    os << "\t\t" << "d[index] = 1;" << std::endl;
    os << "\t\t" << GetDervName(func->name) << "_kernel(";
    first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        os << input.GetName();
        first = false;
    }
    os << ", d, &g);" << std::endl;
    os << "\t\t" << "grad[index] = g;" << std::endl;
    os << "\t}" << std::endl;
    os << "}" << std::endl;
}

// Reverse mode autodiff 
void EmitGrad2(const std::shared_ptr<Function> &func, 
               const ExpressionCPtrVec &wrt, 
               const ExpressionCPtr &dep, 
               std::ostream &os, 
               const bool emitIspc)
{
    EmitHelper helper;
    if (!emitIspc) {
        os << "#include <math.h>" << std::endl;
    }
    os << "static void " << GetDervName(func->name) << "_kernel(";
    bool first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false; 
    }
    if (wrt.size() > 0) {
         if (!first) {
            os << ", ";
        }
        os << c_FloatTypeDecl << "grad[" << wrt.size() << "]";
    }
    os << ") {" << std::endl;

    std::stringstream forwardStream;
    helper.IncTab();
    helper.PrintTab(forwardStream);
    forwardStream << "/* Forward */" << std::endl;

    for (auto &expr : wrt) {
        assert(expr.get() != nullptr);
        helper.Register(expr);
    }

    // std::unordered_map<ExpressionCPtr, ExpressionCPtr> fwdExprs;
    // Forward(func, wrt, fwdExprs);

    // for (auto &expr : wrt) {
    //     helper.Register(fwdExprs[expr]);
    // }
    Emit(helper, func->firstBlock, forwardStream, emitIspc);

    assert(dep.get() != nullptr);
    std::unordered_set<ExpressionCPtr> nonZeroAccId;
    std::stringstream backwardStream;
    if (helper.ExprRegistered(dep)) {
        helper.PrintTab(backwardStream);
        backwardStream << "/* Reverse accumulation */" << std::endl;

        helper.PrintTab(backwardStream);
        backwardStream << "_acc" << helper.GetExprId(dep) << " = 1.0;" << std::endl;
        nonZeroAccId.insert(dep);
        EmitReverse(helper, func->firstBlock, nonZeroAccId, backwardStream, emitIspc);
        int id = 0;
        for (auto &expr : wrt) {
            helper.PrintTab(backwardStream);
            backwardStream << "grad[" << id << "] = _acc" << helper.GetExprId(expr) << ";"
                        << std::endl;
            id++;
        }
    }

    std::stringstream backwardDeclStream;
    auto emitted = helper.GetEmitted();
    if (emitted.size() > 0) {
        helper.PrintTab(os);
        if (emitIspc) {
            os << "uniform ";
        }
        os << c_FloatTypeDecl;
        bool first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (nonZeroAccId.find(expr) != nonZeroAccId.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;
        helper.PrintTab(os);
        os << c_FloatTypeDecl;
        first = true;
        for (auto expr : emitted) {
            if (expr->Type() == ExpressionType::Boolean ||
                expr->Type() == ExpressionType::Constant ||
                expr->Type() == ExpressionType::Variable) {
                continue;
            }
            if (nonZeroAccId.find(expr) == nonZeroAccId.end()) {
                if (!first) {
                    os << ", ";
                }
                os << expr->GetEmitName(helper);
                first = false;
            }
        }
        os << ";" << std::endl;

        helper.PrintTab(backwardDeclStream);
        backwardDeclStream << c_FloatTypeDecl;
        first = true;
        for (auto &expr : wrt) {
            if (nonZeroAccId.find(expr) != nonZeroAccId.end()) {
                continue;
            }
            if (!first) {
                backwardDeclStream << ", ";
            }
            backwardDeclStream << "_acc" << helper.GetExprId(expr) << " = 0";
            first = false;
        }
        for (auto &expr : nonZeroAccId) {
            if (!first) {
                backwardDeclStream << ", ";
            }
            backwardDeclStream << "_acc" << helper.GetExprId(expr) << " = 0";
            first = false;
        }
        backwardDeclStream << ";" << std::endl;
    }
    os << forwardStream.str();
    os << backwardDeclStream.str();
    os << backwardStream.str();
    helper.DecTab();
    os << "}" << std::endl;


    if (emitIspc) {
        os << "export ";
    }
    os << "void " << GetDervName(func->name) << "(";
    first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        if (emitIspc) {
            os << "uniform ";
        }
        os << "const " << input.GetDeclaration();
        first = false;
    }
    os << ", ";
    if (emitIspc) {
        os << "uniform ";
    }
    os << c_FloatTypeDecl << "grad[]";
    os << ") {" << std::endl;
    int dim = (int)wrt.size();
    os << "\t" << c_FloatTypeDecl << "g[" << dim << "] = {0};" << std::endl;
    os << "\t" << GetDervName(func->name) << "_kernel(";
    first = true;
    for (auto &input : func->inputs) {
        if (!first) {
            os << ", ";
        }
        os << input.GetName();
        first = false;
    }
    os << ", g);" << std::endl;
    if (emitIspc) {
        os << "\tforeach (index = 0 ... " << dim << ") {" << std::endl;
    } else {
        os << "\tfor (int index = 0; index < " << dim << "; index++) {" << std::endl;
    }
    os << "\t\t" << "grad[index] = g[index];" << std::endl;
    os << "\t}" << std::endl;
    os << "}" << std::endl;
}

Library::Library(const std::string &path, const std::string &name)
    : m_Path(path), m_Name(name), m_Linked(false) {
    if (path.size() > 0 && path.back() != '/') {
        m_Path += "/";
    }
    std::string libFilepath = m_Path + m_Name + ".so";
    std::ifstream f(libFilepath.c_str());
    if (f.good()) {  // file existed
        m_Handle = dlopen(libFilepath.c_str(), RTLD_LAZY);
        m_Linked = true;
    }
}

Library::~Library() {
    if (m_Linked) {
        dlclose(m_Handle);
    }
}

void Library::RegisterFunc(const std::shared_ptr<Function> func) {
    m_Funcs.push_back(func->name);

    std::string cSourceFilename = m_Path + func->name + ".c";
    std::fstream fs(cSourceFilename.c_str(), std::fstream::out);
    Emit(func, fs);

    std::string cObjFilename = m_Path + func->name + ".o";
    std::string gccCmd =
        std::string("gcc ") + "-O3 -c -fPIC -o " + cObjFilename + " " + cSourceFilename;
    if (std::system(gccCmd.c_str()) != 0) {
        std::cerr << "[Warning] compile failed" << std::endl;
    }
}

void Library::RegisterFunc2(const std::shared_ptr<Function> func) {
    m_Funcs.push_back(func->name);

    std::string cSourceFilename = m_Path + func->name + ".c";
    std::fstream fs(cSourceFilename.c_str(), std::fstream::out);
    Emit(func, fs);

    std::string cObjFilename = m_Path + func->name + ".o";
    std::string gccCmd = 
        std::string("gcc ") + "-O3 -c -fPIC -o " + cObjFilename + " " + cSourceFilename;
    if (std::system(gccCmd.c_str()) != 0) {
        std::cerr << "[Warning] compile failed (MALA) " << std::endl;
    }
}

void Library::RegisterFuncDerv(const std::shared_ptr<Function> func,
                               const ExpressionCPtrVec &wrt,
                               const ExpressionCPtr &dep,
                               const bool emitIspc) {
    std::string dervName = GetDervName(func->name);
    m_Funcs.push_back(dervName);

    std::string ext = emitIspc ? std::string(".ispc") : std::string(".c");
    std::string sourceFilepath = m_Path + dervName + ext;
    std::fstream dfs(sourceFilepath.c_str(), std::fstream::out);
    EmitGradHessian(func, wrt, dep, dfs, emitIspc);

    std::string objFilepath = m_Path + dervName + ".o";
    std::string cmd;
    if (emitIspc) {
        cmd = std::string("ispc ") + "-O3 --math-lib=default --opt=fast-math --woff --pic " +
              sourceFilepath + " -o " + objFilepath;
    } else {
        cmd = std::string("gcc ") + "-Ofast -std=c11 -march=native -c -fPIC -o" + objFilepath +
              " " + sourceFilepath;
    }

    if (std::system(cmd.c_str()) != 0) {
        std::cerr << "[Warning] compile failed" << std::endl;
    }
}

void Library::RegisterFuncDerv2(const std::shared_ptr<Function> func,
                                const ExpressionCPtrVec &wrt,
                                const ExpressionCPtr &dep,
                                const bool emitIspc) 
{
    std::string dervName = GetDervName(func->name);
    m_Funcs.push_back(dervName);

    std::string ext = emitIspc ? std::string(".ispc") : std::string(".c");
    std::string sourceFilepath = m_Path + dervName + ext;
    std::fstream dfs(sourceFilepath.c_str(), std::fstream::out);
    // EmitGrad(func, wrt, dep, dfs, emitIspc);    // Forward mode 
    EmitGrad2(func, wrt, dep, dfs, emitIspc);   // Reverse mode 

    std::string objFilepath = m_Path + dervName + ".o";
    std::string cmd;
    if (emitIspc) {
        cmd = std::string("ispc ") + "-O3 --math-lib=default --opt=fast-math --woff --pic " +
              sourceFilepath + " -o " + objFilepath;
    } else {
        cmd = std::string("gcc ") + "-Ofast -std=c11 -march=native -c -fPIC -o" + objFilepath +
              " " + sourceFilepath;
    }

    if (std::system(cmd.c_str()) != 0) {
        std::cerr << "[Warning] compile failed (MALA) " << std::endl;
    }
}


void Library::Link() {
    std::string libFilepath = m_Path + m_Name + ".so";
    std::string gccCmd =
        std::string("gcc ") + "-march=native -Ofast -shared -fPIC -o " + libFilepath;
    for (auto &funcName : m_Funcs) {
        gccCmd += " " + (m_Path + funcName) + ".o";
    }
    if (std::system(gccCmd.c_str()) != 0) {
        std::cerr << "[Warning] link failed" << std::endl;
    }
    m_Handle = dlopen(libFilepath.c_str(), RTLD_LAZY);
    m_Linked = true;
    // for (auto &funcName : m_Funcs) {
    //     std::remove((m_Path + funcName + ".o").c_str());
    //     std::remove((m_Path + funcName + ".c").c_str());
    //     std::remove((m_Path + funcName + ".ispc").c_str());
    // }
}

void *Library::GetFunc(const std::string &name) const {
    return dlsym(m_Handle, name.c_str());
}

void *Library::GetFuncDerv(const std::string &name) const {
    std::string dervName = GetDervName(name);
    return dlsym(m_Handle, dervName.c_str());
}

}  // chad
