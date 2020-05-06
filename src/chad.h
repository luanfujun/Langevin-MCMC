#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <cassert>
#include <stack>
#include <sstream>
#include <fstream>
#include <dlfcn.h>
#include <cmath>
#include <array>

namespace chad {

class Expression;
class Variable;
class Constant;
class NamedAssignment;
class Sin;
class Cos;
class Boolean;
class CondExpr;
struct Function;
class Argument;

typedef std::shared_ptr<Expression> ExpressionPtr;
typedef std::shared_ptr<const Expression> ExpressionCPtr;
typedef std::vector<ExpressionPtr> ExpressionPtrVec;
typedef std::vector<ExpressionCPtr> ExpressionCPtrVec;
typedef std::shared_ptr<Boolean> BooleanPtr;
typedef std::shared_ptr<const Boolean> BooleanCPtr;
typedef std::shared_ptr<CondExpr> CondExprPtr;
typedef std::shared_ptr<const CondExpr> CondExprCPtr;

extern std::shared_ptr<Function> g_Function;
#if defined(SINGLE_PRECISION)
const std::string c_FloatType = "float";
#else
const std::string c_FloatType = "float";
#endif
const std::string c_FloatTypeDecl = c_FloatType + " ";
const std::string c_FloatTypeConvert = std::string("(") + c_FloatType + ")";

inline ExpressionCPtr operator-(const ExpressionCPtr &expr);
inline ExpressionCPtr square(const ExpressionCPtr &expr);
inline ExpressionCPtr inverse(const ExpressionCPtr &expr);
inline ExpressionCPtr sin(const ExpressionCPtr &expr);
inline ExpressionCPtr cos(const ExpressionCPtr &expr);
inline ExpressionCPtr tan(const ExpressionCPtr &expr);
inline ExpressionCPtr sqrt(const ExpressionCPtr &expr);
inline ExpressionCPtr asin(const ExpressionCPtr &expr);
inline ExpressionCPtr acos(const ExpressionCPtr &expr);
inline ExpressionCPtr fabs(const ExpressionCPtr &expr);
inline ExpressionCPtr exp(const ExpressionCPtr &expr);
inline ExpressionCPtr log(const ExpressionCPtr &expr);
inline ExpressionCPtr pow(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr atan2(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr pow(const ExpressionCPtr &expr0, const float &expr1);
inline ExpressionCPtr pow(const float &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr fmax(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr fmax(const ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr fmax(const float expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr length2d(const ExpressionCPtr &x, const ExpressionCPtr &y);
inline ExpressionCPtr length3d(const ExpressionCPtr &x,
                               const ExpressionCPtr &y,
                               const ExpressionCPtr &z);
inline ExpressionCPtr dot3d(const std::array<ExpressionCPtr, 3> x,
                            const std::array<ExpressionCPtr, 3> y);
inline ExpressionCPtr length4d(const ExpressionCPtr &x,
                               const ExpressionCPtr &y,
                               const ExpressionCPtr &z,
                               const ExpressionCPtr &w);
inline ExpressionCPtr operator+(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator+(const float expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator+(const ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr &operator+=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr &operator+=(ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr operator-(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator-(const float expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator-(const ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr &operator-=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr &operator-=(ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr operator*(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator*(const float expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator*(const ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr &operator*=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr &operator*=(ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr operator/(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator/(const float expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr operator/(const ExpressionCPtr &expr0, const float expr1);
inline ExpressionCPtr &operator/=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline ExpressionCPtr &operator/=(ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr Gt(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Gt(const float expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Gt(const ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr Gte(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Gte(const float expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Gte(const ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr Eq(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Eq(const float expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Eq(const ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr Lte(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Lte(const float expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Lte(const ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr Lt(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Lt(const float expr0, const ExpressionCPtr &expr1);
inline BooleanCPtr Lt(const ExpressionCPtr &expr0, const float expr1);
inline BooleanCPtr And(const BooleanCPtr &expr0, const BooleanCPtr &expr1);

inline std::shared_ptr<Function> BeginFunction(const std::string &name,
                                               const std::vector<Argument> &inputs);
inline void BeginIf(const BooleanCPtr &cond, const std::vector<CondExprCPtr> &outputs);
inline void BeginElseIf(const BooleanCPtr cond);
inline void BeginElse();
inline void EndIf();
inline void SetCondOutput(const ExpressionCPtrVec &exprs);
inline void EndFunction(const std::vector<std::pair<std::string, ExpressionCPtrVec>> &outputs);

void PushExprToBlock(const ExpressionCPtr &expr);

class EmitHelper {
    public:
    EmitHelper() {
        numTab = 0;
    }
    void Register(const ExpressionCPtr &expr) {
        if (m_ExprMap.find(expr) != m_ExprMap.end()) {
            return;
        }
        m_ExprMap[expr] = m_ExprMap.size();
    }
    bool ExprRegistered(const ExpressionCPtr &expr) const {
        auto it = m_ExprMap.find(expr);
        return it != m_ExprMap.end();
    }
    int GetExprId(const ExpressionCPtr &expr) const {
        auto it = m_ExprMap.find(expr);
        if (it == m_ExprMap.end()) {
            throw std::runtime_error("[EmitHelper:GetExprId] expr not found");
        }
        return it->second;
    }
    void SetEmitted(const ExpressionCPtr &expr) {
        m_Emitted.insert(expr);
    }
    bool IsEmitted(const ExpressionCPtr &expr) const {
        return m_Emitted.find(expr) != m_Emitted.end();
    }
    void IncTab() {
        numTab++;
    }
    void DecTab() {
        numTab--;
    }
    void PrintTab(std::ostream &os) const {
        for (int i = 0; i < numTab; i++) {
            os << "\t";
        }
    }

    std::unordered_set<ExpressionCPtr> GetEmitted() const {
        return m_Emitted;
    }

    private:
    std::unordered_map<ExpressionCPtr, int> m_ExprMap;
    std::unordered_set<ExpressionCPtr> m_Emitted;
    int numTab;
};

enum class ExpressionType {
    Variable,
    Constant,
    NamedAssignment,
    Negate,
    Square,
    Inverse,
    Sin,
    Cos,
    Tan,
    Sqrt,
    ASin,
    ACos,
    Exp,
    Log,
    Pow,
    ATan2,
    Add,
    Minus,
    Multiply,
    Divide,
    Length2D,
    Length3D,
    Dot3D,
    Length4D,
    Boolean,
    CondExpr
};

class Expression : public std::enable_shared_from_this<Expression> {
    public:
    virtual ExpressionType Type() const = 0;
    virtual void Register(EmitHelper &helper) const = 0;
    void EmitAll(EmitHelper &helper, std::ostream &os) const {
        if (helper.IsEmitted(shared_from_this())) {
            return;
        }
        for (auto child : Children()) {
            child->EmitAll(helper, os);
        }
        Emit(helper, os);
        helper.SetEmitted(shared_from_this());
    }
    void EmitIfNotEmitted(EmitHelper &helper, std::ostream &os) const {
        if (helper.IsEmitted(shared_from_this())) {
            return;
        }
        Emit(helper, os);
        helper.SetEmitted(shared_from_this());
    }
    virtual void Emit(const EmitHelper &helper, std::ostream &os) const = 0;
    virtual std::string GetEmitName(const EmitHelper &helper) const = 0;
    virtual ExpressionCPtrVec Children() const = 0;
    virtual ExpressionCPtrVec Dervs() const = 0;
    virtual bool IsConstant() const {
        return false;
    }
    virtual float GetConstantVal() const {
        return 0.0;
    }
};

class Variable : public Expression {
    public:
    static std::shared_ptr<const Variable> Create(const std::string &name, const int index) {
        return std::make_shared<Variable>(name, index);
    }
    Variable(const std::string &name, const int index) : m_Name(name), m_Index(index) {
    }
    ExpressionType Type() const {
        return ExpressionType::Variable;
    }
    void Register(EmitHelper &helper) const {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        if (m_Index >= 0) {
            return m_Name + std::string("[") + std::to_string(m_Index) + std::string("]");
        } else {
            return m_Name;
        }
    }
    ExpressionCPtrVec Children() const {
        return {};
    }
    ExpressionCPtrVec Dervs() const {
        return {};
    }

    private:
    std::string m_Name;
    int m_Index;
};

class Constant : public Expression {
    public:
    static std::shared_ptr<const Constant> Create(const float value) {
        return std::make_shared<Constant>(value);
    }
    Constant(const float value) : m_Value(value) {
    }
    ExpressionType Type() const {
        return ExpressionType::Constant;
    }
    void Register(EmitHelper &helper) const {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::to_string(m_Value);
    }
    ExpressionCPtrVec Children() const {
        return {};
    }
    ExpressionCPtrVec Dervs() const {
        return {};
    }
    virtual bool IsConstant() const {
        return true;
    }
    virtual float GetConstantVal() const {
        return m_Value;
    }

    private:
    float m_Value;
};

class NamedAssignment : public Expression {
    public:
    static std::shared_ptr<const NamedAssignment> Create(const std::string &name,
                                                         const int index,
                                                         const ExpressionCPtr &expr) {
        auto ret = std::make_shared<NamedAssignment>(name, index, expr);
        PushExprToBlock(ret);
        return ret;
    }
    NamedAssignment(const std::string &name, const int index, const ExpressionCPtr &expr)
        : m_Name(name), m_Index(index), m_Expr(expr) {
    }
    ExpressionType Type() const {
        return ExpressionType::NamedAssignment;
    }
    void Register(EmitHelper &helper) const {
        m_Expr->Register(helper);
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " << m_Expr->GetEmitName(helper) << ";" << std::endl;
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return m_Name + std::string("[") + std::to_string(m_Index) + std::string("]");
    }
    ExpressionCPtrVec Children() const {
        return {m_Expr};
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(1.0)};
    }

    private:
    std::string m_Name;
    int m_Index;
    ExpressionCPtr m_Expr;
};

class UnaryExpr : public Expression {
    public:
    UnaryExpr(const ExpressionCPtr &expr) : m_Expr(expr) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr->Register(helper);
        helper.Register(shared_from_this());
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this()));
    }
    ExpressionCPtrVec Children() const {
        return {m_Expr};
    }

    protected:
    ExpressionCPtr m_Expr;
};

class Negate : public UnaryExpr {
    public:
    static std::shared_ptr<const Negate> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Negate>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Negate;
    }
    Negate(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = -" << m_Expr->GetEmitName(helper) << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(-1.0)};
    }
};

class Square : public UnaryExpr {
    public:
    static std::shared_ptr<const Square> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Square>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Square;
    }
    Square(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        auto emitName = m_Expr->GetEmitName(helper);
        os << GetEmitName(helper) << " = " << emitName << " * " << emitName << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {2.0 * m_Expr};
    }
};

class Sin : public UnaryExpr {
    public:
    static std::shared_ptr<const Sin> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Sin>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Sin;
    }
    Sin(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = sin(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {cos(m_Expr)};
    }
};

class Cos : public UnaryExpr {
    public:
    static std::shared_ptr<const Cos> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Cos>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Cos;
    }
    Cos(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = cos(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {-sin(m_Expr)};
    }
};

class Tan : public UnaryExpr {
    public:
    static std::shared_ptr<const Tan> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Tan>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Tan;
    }
    Tan(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = tan(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {inverse(square(cos(m_Expr)))};
    }
};

class Sqrt : public UnaryExpr {
    public:
    static std::shared_ptr<const Sqrt> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Sqrt>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Sqrt;
    }
    Sqrt(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = sqrt(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {inverse(2.0 * shared_from_this())};
    }
};

class Inverse : public UnaryExpr {
    public:
    static std::shared_ptr<const Inverse> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Inverse>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Inverse;
    }
    Inverse(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " + c_FloatTypeConvert + "1.0 / ("
           << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {-square(shared_from_this())};
    }
};

class ASin : public UnaryExpr {
    public:
    static std::shared_ptr<const ASin> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<ASin>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::ASin;
    }
    ASin(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = asin(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {inverse(sqrt(1.0 - square(m_Expr)))};
    }
};

class ACos : public UnaryExpr {
    public:
    static std::shared_ptr<const ACos> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<ACos>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::ACos;
    }
    ACos(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = acos(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {-inverse(sqrt(1.0 - square(m_Expr)))};
    }
};

class Exp : public UnaryExpr {
    public:
    static std::shared_ptr<const Exp> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Exp>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Exp;
    }
    Exp(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = exp(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {shared_from_this()};
    }
};

class Log : public UnaryExpr {
    public:
    static std::shared_ptr<const Log> Create(const ExpressionCPtr &expr) {
        auto ret = std::make_shared<Log>(expr);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Log;
    }
    Log(const ExpressionCPtr &expr) : UnaryExpr(expr) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = log(" << m_Expr->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {inverse(m_Expr)};
    }
};

class BinaryExpr : public Expression {
    public:
    BinaryExpr(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1)
        : m_Expr0(expr0), m_Expr1(expr1) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr0->Register(helper);
        m_Expr1->Register(helper);
        helper.Register(shared_from_this());
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this()));
    }
    ExpressionCPtrVec Children() const {
        return {m_Expr0, m_Expr1};
    }

    protected:
    ExpressionCPtr m_Expr0, m_Expr1;
};

class Add : public BinaryExpr {
    public:
    static std::shared_ptr<const Add> Create(const ExpressionCPtr &expr0,
                                             const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Add>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Add;
    }
    Add(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " << m_Expr0->GetEmitName(helper) << "+"
           << m_Expr1->GetEmitName(helper) << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(1.0), Constant::Create(1.0)};
    }
};

class Minus : public BinaryExpr {
    public:
    static std::shared_ptr<const Minus> Create(const ExpressionCPtr &expr0,
                                               const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Minus>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Minus;
    }
    Minus(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " << m_Expr0->GetEmitName(helper) << "-"
           << m_Expr1->GetEmitName(helper) << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(1.0), Constant::Create(-1.0)};
    }
};

class Multiply : public BinaryExpr {
    public:
    static std::shared_ptr<const Multiply> Create(const ExpressionCPtr &expr0,
                                                  const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Multiply>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Multiply;
    }
    Multiply(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " << m_Expr0->GetEmitName(helper) << " * "
           << m_Expr1->GetEmitName(helper) << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {m_Expr1, m_Expr0};
    }
};

class Divide : public BinaryExpr {
    public:
    static std::shared_ptr<const Divide> Create(const ExpressionCPtr &expr0,
                                                const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Divide>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Divide;
    }
    Divide(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = " << m_Expr0->GetEmitName(helper) << " / "
           << m_Expr1->GetEmitName(helper) << ";" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        auto denominator = inverse(m_Expr1);
        return {denominator, -m_Expr0 * square(denominator)};
    }
};


class Pow : public BinaryExpr {
    public:
    static std::shared_ptr<const Pow> Create(const ExpressionCPtr &expr0,
                                             const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Pow>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Pow;
    }
    Pow(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = pow(" << m_Expr0->GetEmitName(helper) << ","
           << m_Expr1->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        return {m_Expr1 * pow(m_Expr0, m_Expr1 - 1.0), shared_from_this() * log(m_Expr1)};
    }
};

class ATan2 : public BinaryExpr {
    public:
    static std::shared_ptr<const ATan2> Create(const ExpressionCPtr &expr0,
                                               const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<ATan2>(expr0, expr1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::ATan2;
    }
    ATan2(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) : BinaryExpr(expr0, expr1) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = atan2(" << m_Expr0->GetEmitName(helper) << ","
           << m_Expr1->GetEmitName(helper) << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        // https://www.wikiwand.com/en/Atan2#/Derivative
        auto invNorm = inverse(square(m_Expr0) + square(m_Expr1));
        return {m_Expr1 * invNorm, -m_Expr0 * invNorm};
    }
};

class TernaryExpr : public Expression {
    public:
    TernaryExpr(const ExpressionCPtr &expr0,
                const ExpressionCPtr &expr1,
                const ExpressionCPtr &expr2)
        : m_Expr0(expr0), m_Expr1(expr1), m_Expr2(expr2) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr0->Register(helper);
        m_Expr1->Register(helper);
        m_Expr2->Register(helper);
        helper.Register(shared_from_this());
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this()));
    }
    ExpressionCPtrVec Children() const {
        return {m_Expr0, m_Expr1, m_Expr2};
    }

    protected:
    ExpressionCPtr m_Expr0, m_Expr1, m_Expr2;
};

class Length2D : public BinaryExpr {
    public:
    static std::shared_ptr<const Length2D> Create(const ExpressionCPtr &x,
                                                  const ExpressionCPtr &y) {
        auto ret = std::make_shared<Length2D>(x, y);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Length2D;
    }
    Length2D(const ExpressionCPtr &x, const ExpressionCPtr &y) : BinaryExpr(x, y) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        auto x = m_Expr0->GetEmitName(helper);
        auto y = m_Expr1->GetEmitName(helper);
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = sqrt(" << x << "*" << x << "+" << y << "*" << y << ");"
           << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        auto invF = inverse(shared_from_this());
        return {m_Expr0 * invF, m_Expr1 * invF};
    }
};

class Length3D : public TernaryExpr {
    public:
    static std::shared_ptr<const Length3D> Create(const ExpressionCPtr &x,
                                                  const ExpressionCPtr &y,
                                                  const ExpressionCPtr &z) {
        auto ret = std::make_shared<Length3D>(x, y, z);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Length3D;
    }
    Length3D(const ExpressionCPtr &x, const ExpressionCPtr &y, const ExpressionCPtr &z)
        : TernaryExpr(x, y, z) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        auto x = m_Expr0->GetEmitName(helper);
        auto y = m_Expr1->GetEmitName(helper);
        auto z = m_Expr2->GetEmitName(helper);
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = sqrt(" << x << "*" << x << "+" << y << "*" << y << "+" << z
           << "*" << z << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        auto invF = inverse(shared_from_this());
        return {m_Expr0 * invF, m_Expr1 * invF, m_Expr2 * invF};
    }
};

class Dot3D : public Expression {
    public:
    static std::shared_ptr<const Dot3D> Create(const std::array<ExpressionCPtr, 3> &v0,
                                               const std::array<ExpressionCPtr, 3> &v1) {
        auto ret = std::make_shared<Dot3D>(v0, v1);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Dot3D;
    }
    Dot3D(const std::array<ExpressionCPtr, 3> &v0, const std::array<ExpressionCPtr, 3> &v1)
        : m_V0(v0), m_V1(v1) {
    }
    void Register(EmitHelper &helper) const {
        for (int i = 0; i < 3; i++) {
            m_V0[i]->Register(helper);
        }
        for (int i = 0; i < 3; i++) {
            m_V1[i]->Register(helper);
        }
        helper.Register(shared_from_this());
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this()));
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        auto x0 = m_V0[0]->GetEmitName(helper);
        auto y0 = m_V0[1]->GetEmitName(helper);
        auto z0 = m_V0[2]->GetEmitName(helper);
        auto x1 = m_V1[0]->GetEmitName(helper);
        auto y1 = m_V1[1]->GetEmitName(helper);
        auto z1 = m_V1[2]->GetEmitName(helper);
        helper.PrintTab(os);

        os << GetEmitName(helper) << " = " << x0 << " * " << x1 << " + " << y0 << " * " << y1
           << " + " << z0 << " * " << z1 << ";" << std::endl;
    }
    ExpressionCPtrVec Children() const {
        return {m_V0[0], m_V0[1], m_V0[2], m_V1[0], m_V1[1], m_V1[2]};
    }
    ExpressionCPtrVec Dervs() const {
        return {m_V1[0], m_V1[1], m_V1[2], m_V0[0], m_V0[1], m_V0[2]};
    }

    private:
    std::array<ExpressionCPtr, 3> m_V0, m_V1;
};

class QuaternaryExpr : public Expression {
    public:
    QuaternaryExpr(const ExpressionCPtr &expr0,
                   const ExpressionCPtr &expr1,
                   const ExpressionCPtr &expr2,
                   const ExpressionCPtr &expr3)
        : m_Expr0(expr0), m_Expr1(expr1), m_Expr2(expr2), m_Expr3(expr3) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr0->Register(helper);
        m_Expr1->Register(helper);
        m_Expr2->Register(helper);
        m_Expr3->Register(helper);
        helper.Register(shared_from_this());
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this()));
    }
    ExpressionCPtrVec Children() const {
        return {m_Expr0, m_Expr1, m_Expr2, m_Expr3};
    }

    protected:
    ExpressionCPtr m_Expr0, m_Expr1, m_Expr2, m_Expr3;
};

class Length4D : public QuaternaryExpr {
    public:
    static std::shared_ptr<const Length4D> Create(const ExpressionCPtr &x,
                                                  const ExpressionCPtr &y,
                                                  const ExpressionCPtr &z,
                                                  const ExpressionCPtr &w) {
        auto ret = std::make_shared<Length4D>(x, y, z, w);
        PushExprToBlock(ret);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Length4D;
    }
    Length4D(const ExpressionCPtr &x,
             const ExpressionCPtr &y,
             const ExpressionCPtr &z,
             const ExpressionCPtr &w)
        : QuaternaryExpr(x, y, z, w) {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
        auto x = m_Expr0->GetEmitName(helper);
        auto y = m_Expr1->GetEmitName(helper);
        auto z = m_Expr2->GetEmitName(helper);
        auto w = m_Expr3->GetEmitName(helper);
        helper.PrintTab(os);
        os << GetEmitName(helper) << " = sqrt(" << x << "*" << x << "+" << y << "*" << y << "+" << z
           << "*" << z << "+" << w << "*" << w << ");" << std::endl;
    }
    ExpressionCPtrVec Dervs() const {
        auto invF = inverse(shared_from_this());
        return {m_Expr0 * invF, m_Expr1 * invF, m_Expr2 * invF, m_Expr3 * invF};
    }
};

class Boolean : public Expression {
    public:
    enum Op { GREATER, GREATER_OR_EQUAL, EQUAL, NOT_EQUAL, LESS_OR_EQUAL, LESS };
    static BooleanCPtr Create(const Op op,
                              const ExpressionCPtr &expr0,
                              const ExpressionCPtr &expr1) {
        auto ret = std::make_shared<Boolean>(op, expr0, expr1);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Boolean;
    }
    Boolean() {
    }
    Boolean(const Op op, const ExpressionCPtr &expr0, const ExpressionCPtr &expr1)
        : m_Op(op), m_Expr0(expr0), m_Expr1(expr1) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr0->Register(helper);
        m_Expr1->Register(helper);
        helper.Register(shared_from_this());
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
    }
    virtual std::string GetEmitName(const EmitHelper &helper) const {
        return m_Expr0->GetEmitName(helper) + OpToString() + m_Expr1->GetEmitName(helper);
    }

    ExpressionCPtrVec Children() const {
        return {m_Expr0, m_Expr1};
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(0.0), Constant::Create(0.0)};
    }

    private:
    std::string OpToString() const {
        switch (m_Op) {
            case GREATER:
                return " > ";
                break;
            case GREATER_OR_EQUAL:
                return " >= ";
                break;
            case EQUAL:
                return " == ";
                break;
            case NOT_EQUAL:
                return " != ";
                break;
            case LESS_OR_EQUAL:
                return " <= ";
                break;
            case LESS:
                return " < ";
                break;
        }
        return "";
    }

    Op m_Op;
    ExpressionCPtr m_Expr0, m_Expr1;
};

class BooleanAnd : public Boolean {
    public:
    static BooleanCPtr Create(const BooleanCPtr &expr0, const BooleanCPtr &expr1) {
        auto ret = std::make_shared<BooleanAnd>(expr0, expr1);
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::Boolean;
    }
    BooleanAnd(const BooleanCPtr &expr0, const BooleanCPtr &expr1)
        : m_Expr0(expr0), m_Expr1(expr1) {
    }
    void Register(EmitHelper &helper) const {
        m_Expr0->Register(helper);
        m_Expr1->Register(helper);
        helper.Register(shared_from_this());
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("(") + m_Expr0->GetEmitName(helper) + std::string(") && (") +
               m_Expr1->GetEmitName(helper) + std::string(")");
    }

    ExpressionCPtrVec Children() const {
        return {m_Expr0, m_Expr1};
    }
    ExpressionCPtrVec Dervs() const {
        return {Constant::Create(0.0), Constant::Create(0.0)};
    }

    private:
    BooleanCPtr m_Expr0, m_Expr1;
};

class CondExpr : public Expression {
    public:
    static std::shared_ptr<const CondExpr> Create() {
        auto ret = std::make_shared<CondExpr>();
        return ret;
    }
    ExpressionType Type() const {
        return ExpressionType::CondExpr;
    }
    CondExpr() {
    }
    void Register(EmitHelper &helper) const {
    }
    void Emit(const EmitHelper &helper, std::ostream &os) const {
    }
    std::string GetEmitName(const EmitHelper &helper) const {
        return std::string("_t") + std::to_string(helper.GetExprId(shared_from_this())) +
               std::string("");
    }

    ExpressionCPtrVec Children() const {
        return m_PossibleExprs;
    }
    ExpressionCPtrVec Dervs() const {
        ExpressionCPtrVec ret;
        for (int i = 0; i < (int)m_PossibleExprs.size(); i++) {
            ret.push_back(Constant::Create(1.0));
        }
        return ret;
    }
    void AddPossibleExpr(const ExpressionCPtr expr) const {
        m_PossibleExprs.push_back(expr);
    }
    ExpressionCPtr GetPossibleExpr(const int index) const {
        return m_PossibleExprs[index];
    }

    private:
    // ugly!
    mutable ExpressionCPtrVec m_PossibleExprs;
};

inline ExpressionCPtr operator-(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(-expr->GetConstantVal());
    }
    return Negate::Create(expr);
}

inline ExpressionCPtr square(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        auto constVal = expr->GetConstantVal();
        return Constant::Create(constVal * constVal);
    }
    return Square::Create(expr);
}

inline ExpressionCPtr inverse(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        auto constVal = expr->GetConstantVal();
        return Constant::Create(1.0 / constVal);
    }
    return Inverse::Create(expr);
}

inline ExpressionCPtr sin(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::sin(expr->GetConstantVal()));
    }
    return Sin::Create(expr);
}

inline ExpressionCPtr cos(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::cos(expr->GetConstantVal()));
    }
    return Cos::Create(expr);
}

inline ExpressionCPtr tan(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::tan(expr->GetConstantVal()));
    }
    return Tan::Create(expr);
}

inline ExpressionCPtr asin(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::asin(expr->GetConstantVal()));
    }
    return ASin::Create(expr);
}

inline ExpressionCPtr acos(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::acos(expr->GetConstantVal()));
    }
    return ACos::Create(expr);
}

inline ExpressionCPtr atan2(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    if (expr0->IsConstant() && expr1->IsConstant()) {
        return Constant::Create(std::atan2(expr0->GetConstantVal(), expr1->GetConstantVal()));
    }
    return ATan2::Create(expr0, expr1);
}

inline ExpressionCPtr sqrt(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::sqrt(expr->GetConstantVal()));
    }
    return Sqrt::Create(expr);
}

inline ExpressionCPtr exp(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::exp(expr->GetConstantVal()));
    }
    return Exp::Create(expr);
}

inline ExpressionCPtr log(const ExpressionCPtr &expr) {
    if (expr->IsConstant()) {
        return Constant::Create(std::log(expr->GetConstantVal()));
    }
    return Log::Create(expr);
}

inline ExpressionCPtr pow(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    if (expr0->IsConstant() && expr1->IsConstant()) {
        return Constant::Create(std::pow(expr0->GetConstantVal(), expr1->GetConstantVal()));
    }
    return Pow::Create(expr0, expr1);
}

inline ExpressionCPtr pow(const ExpressionCPtr &expr0, const float &expr1) {
    return pow(expr0, Constant::Create(expr1));
}

inline ExpressionCPtr pow(const float &expr0, const ExpressionCPtr &expr1) {
    return pow(Constant::Create(expr0), expr1);
}

inline ExpressionCPtr length3d(const ExpressionCPtr &x, const ExpressionCPtr &y) {
    if (x->IsConstant() && y->IsConstant()) {
        auto xVal = x->GetConstantVal();
        auto yVal = y->GetConstantVal();
        return Constant::Create(std::sqrt(xVal * xVal + yVal * yVal));
    }
    return Length2D::Create(x, y);
}

inline ExpressionCPtr length3d(const ExpressionCPtr &x,
                               const ExpressionCPtr &y,
                               const ExpressionCPtr &z) {
    if (x->IsConstant() && y->IsConstant() && z->IsConstant()) {
        auto xVal = x->GetConstantVal();
        auto yVal = y->GetConstantVal();
        auto zVal = z->GetConstantVal();
        return Constant::Create(std::sqrt(xVal * xVal + yVal * yVal + zVal * zVal));
    }
    return Length3D::Create(x, y, z);
}

inline ExpressionCPtr dot3d(const std::array<ExpressionCPtr, 3> x,
                            const std::array<ExpressionCPtr, 3> y) {
    return Dot3D::Create(x, y);
}

inline ExpressionCPtr length4d(const ExpressionCPtr &x,
                               const ExpressionCPtr &y,
                               const ExpressionCPtr &z,
                               const ExpressionCPtr &w) {
    if (x->IsConstant() && y->IsConstant() && z->IsConstant() && w->IsConstant()) {
        auto xVal = x->GetConstantVal();
        auto yVal = y->GetConstantVal();
        auto zVal = z->GetConstantVal();
        auto wVal = w->GetConstantVal();
        return Constant::Create(std::sqrt(xVal * xVal + yVal * yVal + zVal * zVal + wVal * wVal));
    }
    return Length4D::Create(x, y, z, w);
}

inline ExpressionCPtr fabs(const ExpressionCPtr &expr) {
    auto ret = CondExpr::Create();
    BeginIf(Gte(expr, 0.0), {ret});
    { SetCondOutput({expr}); }
    BeginElse();
    { SetCondOutput({-expr}); }
    EndIf();
    return ret;
}

inline ExpressionCPtr fmax(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    auto ret = CondExpr::Create();
    BeginIf(Gte(expr0, expr1), {ret});
    { SetCondOutput({expr0}); }
    BeginElse();
    { SetCondOutput({expr1}); }
    EndIf();
    return ret;
}

inline ExpressionCPtr fmax(const ExpressionCPtr &expr0, const float expr1) {
    return fmax(expr0, Constant::Create(expr1));
}

inline ExpressionCPtr fmax(const float expr0, const ExpressionCPtr &expr1) {
    return fmax(Constant::Create(expr0), expr1);
}

inline ExpressionCPtr operator+(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    if (expr0->IsConstant() && expr0->GetConstantVal() == 0.0) {
        return expr1;
    }
    if (expr1->IsConstant() && expr1->GetConstantVal() == 0.0) {
        return expr0;
    }
    if (expr0->IsConstant() && expr1->IsConstant()) {
        return Constant::Create(expr0->GetConstantVal() + expr1->GetConstantVal());
    }
    return Add::Create(expr0, expr1);
}
inline ExpressionCPtr operator+(const ExpressionCPtr &expr0, const float expr1) {
    return expr0 + Constant::Create(expr1);
}
inline ExpressionCPtr operator+(const float expr0, const ExpressionCPtr &expr1) {
    return Constant::Create(expr0) + expr1;
}
inline ExpressionCPtr &operator+=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    expr0 = expr0 + expr1;
    return expr0;
}
inline ExpressionCPtr &operator+=(ExpressionCPtr &expr0, const float expr1) {
    expr0 = expr0 + expr1;
    return expr0;
}
inline ExpressionCPtr operator-(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    if (expr0->IsConstant() && expr0->GetConstantVal() == 0.0) {
        return -expr1;
    }
    if (expr1->IsConstant() && expr1->GetConstantVal() == 0.0) {
        return expr0;
    }
    if (expr0->IsConstant() && expr1->IsConstant()) {
        return Constant::Create(expr0->GetConstantVal() - expr1->GetConstantVal());
    }
    return Minus::Create(expr0, expr1);
}
inline ExpressionCPtr operator-(const ExpressionCPtr &expr0, const float expr1) {
    return expr0 - Constant::Create(expr1);
}
inline ExpressionCPtr operator-(const float expr0, const ExpressionCPtr &expr1) {
    return Constant::Create(expr0) - expr1;
}
inline ExpressionCPtr &operator-=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    expr0 = expr0 - expr1;
    return expr0;
}
inline ExpressionCPtr &operator-=(ExpressionCPtr &expr0, const float expr1) {
    expr0 = expr0 - expr1;
    return expr0;
}

inline ExpressionCPtr operator*(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    if (expr0->IsConstant()) {
        if (expr0->GetConstantVal() == 0.0) {
            return Constant::Create(0.0);
        } else if (expr0->GetConstantVal() == 1.0) {
            return expr1;
        } else if (expr0->GetConstantVal() == -1.0) {
            return -expr1;
        }
    }
    if (expr1->IsConstant()) {
        if (expr1->GetConstantVal() == 0.0) {
            return Constant::Create(0.0);
        } else if (expr1->GetConstantVal() == 1.0) {
            return expr0;
        } else if (expr1->GetConstantVal() == -1.0) {
            return -expr0;
        }
    }
    if (expr0->IsConstant() && expr1->IsConstant()) {
        return Constant::Create(expr0->GetConstantVal() * expr1->GetConstantVal());
    }
    return Multiply::Create(expr0, expr1);
}
inline ExpressionCPtr operator*(const ExpressionCPtr &expr0, const float expr1) {
    return expr0 * Constant::Create(expr1);
}
inline ExpressionCPtr operator*(const float expr0, const ExpressionCPtr &expr1) {
    return Constant::Create(expr0) * expr1;
}
inline ExpressionCPtr &operator*=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    expr0 = expr0 * expr1;
    return expr0;
}
inline ExpressionCPtr &operator*=(ExpressionCPtr &expr0, const float expr1) {
    expr0 = expr0 * expr1;
    return expr0;
}

inline ExpressionCPtr operator/(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Divide::Create(expr0, expr1);
}
inline ExpressionCPtr operator/(const ExpressionCPtr &expr0, const float expr1) {
    return expr0 * (1.0 / expr1);
}
inline ExpressionCPtr operator/(const float expr0, const ExpressionCPtr &expr1) {
    return expr0 * inverse(expr1);
}
inline ExpressionCPtr &operator/=(ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    expr0 = expr0 / expr1;
    return expr0;
}
inline ExpressionCPtr &operator/=(ExpressionCPtr &expr0, const float expr1) {
    expr0 = expr0 / expr1;
    return expr0;
}

inline BooleanCPtr Gt(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Boolean::Create(Boolean::GREATER, expr0, expr1);
}
inline BooleanCPtr Gt(const ExpressionCPtr &expr0, const float expr1) {
    return Gt(expr0, Constant::Create(expr1));
}
inline BooleanCPtr Gt(const float expr0, const ExpressionCPtr &expr1) {
    return Gt(Constant::Create(expr0), expr1);
}
inline BooleanCPtr Gte(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Boolean::Create(Boolean::GREATER_OR_EQUAL, expr0, expr1);
}
inline BooleanCPtr Gte(const ExpressionCPtr &expr0, const float expr1) {
    return Gte(expr0, Constant::Create(expr1));
}
inline BooleanCPtr Gte(const float expr0, const ExpressionCPtr &expr1) {
    return Gte(Constant::Create(expr0), expr1);
}
inline BooleanCPtr Eq(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Boolean::Create(Boolean::EQUAL, expr0, expr1);
}
inline BooleanCPtr Eq(const ExpressionCPtr &expr0, const float expr1) {
    return Eq(expr0, Constant::Create(expr1));
}
inline BooleanCPtr Eq(const float expr0, const ExpressionCPtr &expr1) {
    return Eq(Constant::Create(expr0), expr1);
}
inline BooleanCPtr Lte(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Boolean::Create(Boolean::LESS_OR_EQUAL, expr0, expr1);
}
inline BooleanCPtr Lte(const ExpressionCPtr &expr0, const float expr1) {
    return Lte(expr0, Constant::Create(expr1));
}
inline BooleanCPtr Lte(const float expr0, const ExpressionCPtr &expr1) {
    return Lte(Constant::Create(expr0), expr1);
}
inline BooleanCPtr Lt(const ExpressionCPtr &expr0, const ExpressionCPtr &expr1) {
    return Boolean::Create(Boolean::LESS, expr0, expr1);
}
inline BooleanCPtr Lt(const ExpressionCPtr &expr0, const float expr1) {
    return Lt(expr0, Constant::Create(expr1));
}
inline BooleanCPtr Lt(const float expr0, const ExpressionCPtr &expr1) {
    return Lt(Constant::Create(expr0), expr1);
}
inline BooleanCPtr And(const BooleanCPtr &expr0, const BooleanCPtr &expr1) {
    return BooleanAnd::Create(expr0, expr1);
}

class Argument {
    public:
    Argument(const std::string &name, int size = 1) : m_Name(name) {
        for (int i = 0; i < size; i++) {
            m_Exprs.push_back(Variable::Create(name, size == 1 ? -1 : i));
        }
    }
    std::string GetDeclaration() const {
        if (m_Exprs.size() == 1) {
            return c_FloatTypeDecl + m_Name;
        } else {
            return c_FloatTypeDecl + m_Name + "[" + std::to_string(m_Exprs.size()) + "]";
        }
    }
    std::string GetName() const {
        return m_Name;
    }
    ExpressionCPtr GetExpr(int index = 0) const {
        return m_Exprs[index];
    }
    ExpressionCPtrVec GetExprVec() const {
        return m_Exprs;
    }

    private:
    std::string m_Name;
    ExpressionCPtrVec m_Exprs;
};

struct CFGSplit;

struct CFGBlock {
    std::vector<ExpressionCPtr> exprs;
    std::shared_ptr<CFGSplit> next;
};

struct CFGSplit {
    std::vector<BooleanCPtr> conditions;
    std::vector<std::shared_ptr<CFGBlock>> children;
    std::vector<CondExprCPtr> outputs;
    std::shared_ptr<CFGBlock> next;
};

struct Function {
    std::shared_ptr<CFGBlock> firstBlock;
    std::shared_ptr<CFGBlock> curBlock;
    std::stack<std::shared_ptr<CFGSplit>> splitStack;

    std::vector<Argument> inputs;
    std::vector<std::pair<std::string, ExpressionCPtrVec>> outputs;
    std::string name;
};

inline std::shared_ptr<Function> BeginFunction(const std::string &name,
                                               const std::vector<Argument> &inputs) {
    auto func = std::make_shared<Function>();
    g_Function = func;
    func->firstBlock = func->curBlock = std::make_shared<CFGBlock>();
    func->name = name;
    func->inputs = inputs;
    return func;
}

inline void BeginIf(const BooleanCPtr &cond, const std::vector<CondExprCPtr> &outputs) {
    auto newSplit = std::make_shared<CFGSplit>();
    g_Function->curBlock->next = newSplit;
    g_Function->splitStack.push(newSplit);
    auto newBlock = std::make_shared<CFGBlock>();
    newSplit->conditions.push_back(cond);
    newSplit->children.push_back(newBlock);
    newSplit->outputs = outputs;
    g_Function->curBlock = newBlock;
}
inline void BeginElseIf(const BooleanCPtr cond) {
    auto newBlock = std::make_shared<CFGBlock>();
    g_Function->splitStack.top()->conditions.push_back(cond);
    g_Function->splitStack.top()->children.push_back(newBlock);
    g_Function->curBlock = newBlock;
}
inline void BeginElse() {
    auto newBlock = std::make_shared<CFGBlock>();
    // Still push a nullptr condition for vector size consistency
    g_Function->splitStack.top()->conditions.push_back(BooleanCPtr());
    g_Function->splitStack.top()->children.push_back(newBlock);
    g_Function->curBlock = newBlock;
}
inline void EndIf() {
    auto newBlock = std::make_shared<CFGBlock>();
    g_Function->splitStack.top()->next = newBlock;
    g_Function->splitStack.pop();
    g_Function->curBlock = newBlock;
}

inline void EndFunction(const std::vector<std::pair<std::string, ExpressionCPtrVec>> &outputs) {
    if (g_Function->splitStack.size() != 0) {
        throw std::runtime_error("[chad.h:EndFunction] Non-empty splitStack");
    }
    g_Function->curBlock = std::shared_ptr<CFGBlock>();
    g_Function->outputs = outputs;
}

inline void SetCondOutput(const ExpressionCPtrVec &exprs) {
    if (g_Function->splitStack.size() == 0) {
        throw std::runtime_error("[chad.h:SetCondOutput] Not inside if");
    }
    if (exprs.size() != g_Function->splitStack.top()->outputs.size()) {
        std::cerr << "exprs.size():" << exprs.size() << std::endl;
        std::cerr << "outputs.size():" << g_Function->splitStack.top()->outputs.size() << std::endl;
        throw std::runtime_error("[chad.h:SetCondOutput] Size does not match");
    }
    for (int i = 0; i < (int)exprs.size(); i++) {
        g_Function->splitStack.top()->outputs[i]->AddPossibleExpr(exprs[i]);
    }
}


inline void PushExprToBlock(const ExpressionCPtr &expr) {
    if (g_Function.get() != nullptr && g_Function->curBlock.get() != nullptr) {
        g_Function->curBlock->exprs.push_back(expr);
    }
}

void Emit(const std::shared_ptr<Function> &func, std::ostream &os);

void EmitGradHessian(const std::shared_ptr<Function> &func,
                     const ExpressionCPtrVec &wrt,
                     const ExpressionCPtr &dep,
                     std::ostream &os,
                     const bool emitIspc);

inline std::string GetDervName(const std::string &name) {
    return name + "_derv";
}

typedef void *lib_t;

class Library {
    public:
    Library(const std::string &path, const std::string &name);
    virtual ~Library();
    void RegisterFunc(const std::shared_ptr<Function> func);
    void RegisterFuncDerv(const std::shared_ptr<Function> func,
                          const ExpressionCPtrVec &wrt,
                          const ExpressionCPtr &dep,
                          const bool emitIspc = true);
    void RegisterFunc2(const std::shared_ptr<Function> func);
    void RegisterFuncDerv2(const std::shared_ptr<Function> func,
                           const ExpressionCPtrVec &wrt,
                           const ExpressionCPtr &dep,
                           const bool emitIspc = true);
    inline bool IsLinked() const {
        return m_Linked;
    }
    void Link();
    void *GetFunc(const std::string &name) const;
    void *GetFuncDerv(const std::string &name) const;

    private:
    std::string m_Path;
    std::string m_Name;
    std::vector<std::string> m_Funcs;
    lib_t m_Handle;
    bool m_Linked;
};

}  // namespace chad
